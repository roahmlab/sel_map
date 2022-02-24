/**
 * @file ElemArray.hpp
 * @author Adam Li (adamli@umich.edu)
 * @date 2021-11-11
 * @brief The template class definition for static and dynamic size ElemArray.
 * 
 * ElemArray is written to behave somewhat similarly to a standard vector
 * at the highest level, but it's really just a custom datatype made
 * specifically for the mesh components of sel_map. Raw memory access is allowed
 * for mapping to eigen (an later a simple function to do this is probably going
 * to be added). The goal is to try to optimize the use of memory and keep things
 * aligned for vectorizing and automatic optimization.
 * 
 */
#pragma once

// First include the base definition, which we will extend here
#include "ElemArrayBase.hpp"
// Include the Eigen memory definitions, which allows for aligned allocation
#include <Eigen/Core>
#include <Eigen/Dense>

// And now include the rest of the standard library needed to make this work
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <algorithm>
#include <cassert>

namespace sel_map::core {
    //Forward declare ElemArray helpers.
    template <typename ElemArray_t, typename Tprimative_t, std::size_t Tsize_el>
    class ElemArrayHelper;
    // So code is more readable
    static const int ElemArrayDynamic = 0;
    // If Tsize_el is set to zero, this is effectively a dynamic length el vector
    // an extra typedef for ElemArrayDyn is made just for that case at the end.
    // I lied, typedefs with template parameters is not a thing.
    template <typename Tprimative_t, std::size_t Tsize_el>
    class ElemArray : public ElemArrayBase {

        // Helpers for static/dynamic functions
        friend class ElemArrayHelper<ElemArray<Tprimative_t, Tsize_el>, Tprimative_t, Tsize_el>;
        using Helper = ElemArrayHelper<ElemArray<Tprimative_t, Tsize_el>, Tprimative_t, Tsize_el>;

        public:
        // Extra eigen intercompat for nanoflann
        typedef Tprimative_t Scalar;
        constexpr static const int ColsAtCompileTime = Tsize_el;

        //==============================================//
        //       EIGEN TYPEDEFS FOR READABILITY         //
        //==============================================//
        // Make life easier by typedef'ing the vectors and matrix here for static and dynamic case
        typedef typename std::conditional<Tsize_el == ElemArrayDynamic,
                    Eigen::Matrix<Tprimative_t, Eigen::Dynamic, 1>, // Dynamic Row Vec (as eigen col vec)
                    Eigen::Matrix<Tprimative_t, Tsize_el, 1>        // Static Row Vec (as eigen col vec)
                    >::type EigenRowVec_t;

        // All column vectors will be dynamic because length can change
        typedef Eigen::Matrix<Tprimative_t, Eigen::Dynamic, 1> EigenColVec_t;

        // Matrix typedef
        typedef typename std::conditional<Tsize_el == ElemArrayDynamic,
                    Eigen::Matrix<Tprimative_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,    //Matrix with dynamic row size
                    Eigen::Matrix<Tprimative_t, Eigen::Dynamic, Tsize_el, Eigen::RowMajor>           //Matrix with static row size
                    >::type EigenMatrix_t;

        // Array typedef
        typedef typename std::conditional<Tsize_el == ElemArrayDynamic,
                    Eigen::Array<Tprimative_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,    //Array with dynamic row size
                    Eigen::Array<Tprimative_t, Eigen::Dynamic, Tsize_el, Eigen::RowMajor>           //Array with static row size
                    >::type EigenArray_t;

        // Chose the right innerstride
        typedef typename std::conditional<Tsize_el == ElemArrayDynamic,
                    Eigen::InnerStride<>,           //Dynamic inner stride
                    Eigen::InnerStride<Tsize_el>    //Static inner stride
                    >::type EigenInnerStride_t;

        //==============================================//
        //       HELPER OBJECTS / NESTED CLASSES        //
        //               IMPLEMENTATION                 //
        //==============================================//
        class Iterator;
        // Each element in this ElemArray (used as return type)
        // Necessary operators are added to this as needed
        class Elem : public ElemArrayBase::ElemBase {
            friend class Iterator;
            friend class ElemArray;
            protected:
            // The size of a singular element
            std::size_t _size_el;
            bool temporary;
            
            // Main constructor, protected
            Elem(std::size_t size_el = Tsize_el) : _size_el(size_el), temporary(false) {
                // If you see this, you might be trying to directly create an Elem
                // This isn't valid though. Elem is simply a middleman object for access.
                // Also something had to go terribly wrong, because this is protected.
                // Nevermind
                // Apparently MinMax needs this.
                //assert(size_el && "If you see this, something broke! Dynamic Elem's must have size_el greater than 0!");
            };

            public:
            // This object isn't supposed to be constructable from scratch!
            // Copy things
            Elem(const Elem& other) {
                if (Tsize_el && Tsize_el != other._size_el)
                    throw std::invalid_argument("Element sizes must match for static copy of Elem Accessor");
                this->_size_el = other._size_el;
                temporary = true;
                _elem = (void*)new Tprimative_t[_size_el];
                std::copy(other.begin(), other.end(), begin());
            };
            Elem& operator=(const Elem& rhs) {
                if (this != &rhs){
                    if (Tsize_el && Tsize_el != rhs._size_el)
                        throw std::invalid_argument("Element sizes must match for static Elem Copy Assignment");
                    this->_size_el = rhs._size_el;
                    this->_elem = rhs._elem;
                    //std::copy(rhs.begin(), rhs.end(), begin());
                }
                return *this;
            };
            
            // Move things (this is just masking a copy)
            Elem(Elem&& other){
                if (Tsize_el && Tsize_el != other._size_el)
                    throw std::invalid_argument("Element sizes must match for static move of Elem Accessor");
                this->_size_el = other._size_el;
                temporary = true;
                _elem = (void*)new Tprimative_t[_size_el];
                std::swap_ranges(other.begin(), other.end(), begin());
            };
            Elem& operator=(Elem&& rhs){
                if (this != &rhs){
                    swap(rhs);
                }
                return *this;
            };
            ~Elem(){
                if (temporary) delete[] (Tprimative_t*)_elem;
            }

            // Typed, unchecked accessor for the internal element pointer
            Tprimative_t* ptr(){
                return ptr(_elem);
            };
            Tprimative_t* const_ptr() const {
                return const_ptr(_elem);
            };

            // Typed, unchecked accessor for arbitrary pointer of same type
            Tprimative_t* ptr(void* elem){
                return (Tprimative_t*)elem;
            };
            Tprimative_t* const_ptr(void* elem) const {
                return (Tprimative_t*)elem;
            };

            // Typed, unchecked accessor to next Elem following structure
            // Assumes the same exact types along bounds
            Tprimative_t* shift(int pos){
                return shift(_elem, pos);
            };
            Tprimative_t* const_shift(int pos) const{
                return const_shift(_elem, pos);
            };
            
            // Typed, unchecked accessor to next Elem following structure
            // Assumes the same exact types for arbitrary pointer of same type
            Tprimative_t* shift(const void* elem, int pos){
                return ((Tprimative_t*)elem) + pos*_size_el;
            };
            Tprimative_t* const_shift(const void* elem, int pos) const{
                return ((Tprimative_t*)elem) + pos*_size_el;
            };

            // Compute the wraparound index and return the relevant element
            Tprimative_t& operator[](int idx){
                return ptr()[((idx % int(_size_el)) + _size_el) % _size_el];
            };
            Tprimative_t operator[](int idx) const{
                return (*this)[idx];
            };

            // Get the mapped Eigen Vector relating to the Elem
            Eigen::Map<EigenRowVec_t> toEigen(){
                return Eigen::Map<EigenRowVec_t>(ptr(), _size_el);
            };

            // Gets pointers mapped to begin and end of elem
            Tprimative_t* begin(){
                return ptr();
            };
            Tprimative_t* end(){
                return shift(1);
            };
            Tprimative_t* begin() const{
                return const_ptr();
            };
            Tprimative_t* end() const{
                return const_shift(1);
            };
            
            //Swap function
            void swap(Elem& other) {
                if (other._size_el != _size_el)
                    throw std::invalid_argument("Element sizes must match for swap");
                std::swap_ranges(other.begin(), other.end(), begin());
            }

            // Size accessor
            std::size_t size() const{ return _size_el; };
        };

        //Custom iterator for use with STL algorithms
        struct Iterator
        {
            // Thank you https://internalpointers.com/post/writing-custom-iterators-modern-cpp
            // https://stackoverflow.com/questions/12092448/code-for-a-basic-random-access-iterator-based-on-pointers/31886483
            using iterator_category = std::random_access_iterator_tag;
            using difference_type   = std::ptrdiff_t;
            using value_type        = Elem;
            using pointer           = Elem*;
            using reference         = Elem&;

            public:

            Iterator(std::size_t size_el = 0) : internal_elem(size_el) { internal_elem._elem =  nullptr; };
            Iterator(Tprimative_t* ptr, std::size_t size_el = 0) : internal_elem(size_el) { internal_elem._elem = (void*)ptr; };
            Iterator(const Iterator& ptr) : internal_elem(ptr.internal_elem._size_el) { internal_elem._elem = ptr.internal_elem._elem; };

            inline Iterator& operator+=(difference_type rhs) {internal_elem._elem = (void*) internal_elem.shift(rhs); return *this;};
            inline Iterator& operator-=(difference_type rhs) {internal_elem._elem = (void*) internal_elem.shift(-rhs); return *this;};
            inline reference operator*() {return internal_elem;};
            inline pointer operator->() const {return &internal_elem;};
            inline reference operator[](difference_type rhs) const {value_type ret; ret._elem = (void*) internal_elem.shift(internal_elem._elem, rhs); return ret;};

            inline Iterator& operator++() {internal_elem._elem = (void*) internal_elem.shift(1); return *this;};
            inline Iterator& operator--() {internal_elem._elem = (void*) internal_elem.shift(-1); return *this;};
            inline Iterator operator++(int) {Iterator tmp(*this); ++(*this); return tmp;};
            inline Iterator operator--(int) {Iterator tmp(*this); --(*this); return tmp;};

            inline difference_type operator-(const Iterator& rhs) const
                {return ((int)(internal_elem.const_ptr()-rhs.internal_elem.const_ptr())) / internal_elem.size();};
            inline Iterator operator+(difference_type rhs) const {return Iterator(internal_elem.const_shift(internal_elem._elem, rhs), internal_elem._size_el);};
            inline Iterator operator-(difference_type rhs) const {return Iterator(internal_elem.const_shift(internal_elem._elem, -rhs), internal_elem._size_el);};
            friend inline Iterator operator+(difference_type lhs, const Iterator& rhs) {return Iterator(rhs.internal_elem.shift(rhs.internal_elem._elem, lhs), rhs.internal_elem._size_el);};
            /*friend inline Iterator operator-(difference_type lhs, const Iterator& rhs) {return Iterator((Tprimative_t*)(lhs*internal_elem.size()) - Elem::pts(rhs.internal_elem.elem));};*/
    

            inline bool operator==(const Iterator& rhs) const {return internal_elem._elem == rhs.internal_elem._elem;};
            inline bool operator!=(const Iterator& rhs) const {return internal_elem._elem != rhs.internal_elem._elem;};
            inline bool operator>(const Iterator& rhs) const {return internal_elem._elem > rhs.internal_elem._elem;};
            inline bool operator<(const Iterator& rhs) const {return internal_elem._elem < rhs.internal_elem._elem;};
            inline bool operator>=(const Iterator& rhs) const {return internal_elem._elem >= rhs.internal_elem._elem;};
            inline bool operator<=(const Iterator& rhs) const {return internal_elem._elem <= rhs.internal_elem._elem;};
            
            friend bool operator== (const Iterator& a, const Iterator& b) { return a.internal_elem._elem == b.internal_elem._elem; };
            friend bool operator!= (const Iterator& a, const Iterator& b) { return a.internal_elem._elem != b.internal_elem._elem; };

            void swap(Iterator& other) {
                internal_elem.swap(other.internal_elem);
            }
        private:
            value_type internal_elem;
        };

        protected:
        //==============================================//
        //            PROTECTED RAW STORAGE             //
        //==============================================//
        // The raw storage for the N-length array of elements
        // The total size of this will be N*size_el.
        //Tprimative_t *arr;
        void* arr;


        private:
        //==============================================//
        //          PRIVATE HELPER FUNCTIONS            //
        //      LOW LEVEL MEMORY SPECIALIZATIONS        //
        //             FOWARD DECLARATIONS              //
        //==============================================//
        /**
         * @brief sizeof wrapper. Returns the sizeof(Tprimative_t)*size_el
         * 
         */
        unsigned int _sizeof(){
            return Helper::_sizeof(this);
        }

        /**
         * @brief Returns the apropriate index to start a row.
         * 
         * @param idx row index to get the raw index of
         * @return unsigned int 
         */
        unsigned int rowIdx(unsigned int idx){
            return Helper::rowIdx(this, idx);
        }

        /**
         * @brief Wrapper for malloc
         * 
         * @param new_size
         *      New length of the internal vector (in units of Elem)
         */
        void allocArr(std::size_t new_size){
            //arr = (Tprimative_t*) std::malloc(_sizeof()*(new_size+1));
            //arr = (void*) std::malloc(_sizeof()*(new_size+1));
            arr = (void*) Eigen::internal::aligned_malloc(_sizeof()*(new_size+1));
        };

        /**
         * @brief Wrapper for realloc
         * 
         * @param new_size
         *      New length of the internal vector (in units of Elem)
         */
        void reallocArr(std::size_t new_size) {
            //arr = (Tprimative_t*) std::realloc(arr, _sizeof()*(new_size+1));
            //arr = (void*) std::realloc(arr, _sizeof()*(new_size+1));
            arr = (void*) Eigen::internal::aligned_realloc(arr, _sizeof()*(new_size+1), _sizeof()*(length_real+1));
        };

        /**
         * @brief Wrapper for memcpy
         * 
         * @param start
         *      Start position in Elem to copy to
         * @param length
         *      # of Elem to copy
         * @param from_arr 
         *      Arr to copy from
         */
        void memcpyArr(std::size_t start, std::size_t length, void* from_arr){
            std::memcpy(dptr(rowIdx(start)), from_arr, _sizeof()*length);
        };

        //==============================================//
        //          PRIVATE HELPER FUNCTIONS            //
        //       HIGHER LEVEL CODE DEDUPLICATION        //
        //==============================================//
        // helper function to copy arr and reduce code duplication
        /**
         * @brief Copies/append from add_arr to arr with copy_len Elem's
         * 
         * @param add_arr
         *      Pointer to the start of the new_arr data
         * @param copy_len 
         *      Number of Elem's to copy
         */
        void copyAppendArr(void* add_arr, std::size_t copy_len){
            // Create the memory needed, if needed
            if (!arr){
                allocArr(copy_len);
                length_real = copy_len;
            }

            // If there isn't enough space at the end to add, then just get space
            else if (length_real - len < copy_len) {
                // Use realloc to try to expand the same space of memory.
                reallocArr(len + copy_len);
                length_real = len + copy_len;
            }

            // Make sure the allocations worked
            if (!arr) throw std::bad_alloc();

            // Then just copy the memory
            memcpyArr(len, copy_len, add_arr);
            len += copy_len;
        };

        /**
         * @brief Copies from ElemArrayBase to this ElemArray.
         * Reduces code duplication
         * 
         * @param toCopy ElemArrayBase to copy from
         */
        void copyAll(const ElemArrayBase& toCopy){
            // Copy only the relevant length & size
            length_real = len = toCopy.len;
            size_el = toCopy.elem_size();
            arr = nullptr;

            // If there's anything worth copying, then allocate (and throw bad alloc if it fails)
            if(len != 0){
                allocArr(len);
                if (!arr) throw std::bad_alloc();

                // and copy the data
                memcpyArr(0, len, (Tprimative_t*)toCopy.getRawMemory());
            }
        };

        /**
         * @brief Swaps the key parameters between the two ElemArray's
         * 
         * @param toSwap 
         */
        void swapAll(ElemArrayBase& toSwap){
            std::swap(size_el, toSwap.size_el);
            std::swap(len, toSwap.len);
            std::swap(length_real, toSwap.length_real);
            // void* tmp = toSwap.getRawMemory();
            // toSwap.getRawMemory() = (void*)arr;
            // arr = (Tprimative_t*)tmp;
            std::swap(arr, toSwap.getRawMemory());
        }


        public:
        //==============================================//
        //          CONSTRUCTION/DESTRUCTION            //
        //      + RELATED OPERATORS & UTIL FUNCS        //
        //==============================================//
        // Deletes the memory used by the array & resets everything to 0
        /**
         * @brief Clears the memory used by the array, setting lengths to 0.
         * 
         * This doesn't change the size_el, and the ElemArray still retains the same
         * elem_size for other operations like append.
         * 
         */
        void clear() {
            if (arr) {
                //std::free(arr);
                Eigen::internal::aligned_free(arr);
                arr = nullptr;
            }
            length_real = len = 0;
        };

        /**
         * @brief Returns the length of a singular element
         * 
         * @return std::size_t size of an element
         */
        std::size_t elem_size() const{
            return Helper::elem_size(this);
        }
        void resize_elem(std::size_t new_size_el) {
            if (Tsize_el)
                throw std::invalid_argument("Static ElemArray's cannot be resized!");
            // invalidate the entire elemArray
            clear();
            size_el = new_size_el;
        }
        
        /**
         * @brief Provides raw memory access to the internal storage. (Element-major)
         * 
         * @return void*& generic pointer to the internal storage
         */
        void*& getRawMemory() { return arr; };
        void* getRawMemory() const { return (void*)arr; };

        /**
         * @brief Construct a new ElemArray object.
         * 
         * @param size_el
         *      If using a static ElemArray, this does nothing. If using a dynamic
         *      ElemArray, or an ElemArrayDyn object, this sets the size of a single element.
         * 
         */
        ElemArray(std::size_t size_el = Tsize_el)
                  : ElemArrayBase(size_el), arr(nullptr)
        {
            // Default constructor, make sure everything is zeros except for size_el.
            if (!size_el)
                throw std::invalid_argument("Dynamic ElemArray's must have an element size greater than 0!");
        };
        
        // Copy things
        /**
         * @brief Generic copy constructor
         * 
         * @param other Base ElemArray object to copy.
         */
        ElemArray(const ElemArrayBase& other) {
            // Check validity
            if (Tsize_el && Tsize_el != other.elem_size())
                throw std::invalid_argument("Element sizes must match to copy to static ElemArray's.");

            // Copy
            copyAll(other);
        };
        /**
         * @brief Specialized copy constructor
         * 
         * @param other ElemArray object to copy.
         */
        ElemArray(const ElemArray<Tprimative_t, Tsize_el>& other) { copyAll(other); };
        /**
         * @brief Generic copy assignment
         * 
         * @param rhs Base ElemArray object to copy.
         * @return self
         */
        ElemArray<Tprimative_t, Tsize_el>& operator=(const ElemArrayBase& rhs) {
            if (this != &rhs){
                // Check validity
                if (Tsize_el && Tsize_el != rhs.elem_size())
                    throw std::invalid_argument("Element sizes must match to copy to static ElemArray's.");
                // Clear the current one
                clear();
                copyAll(rhs);
            }
            return *this;
        };
        /**
         * @brief Specialized copy assignment
         * 
         * @param rhs ElemArray object to copy.
         * @return self
         */
        ElemArray<Tprimative_t, Tsize_el>& operator=(const ElemArray<Tprimative_t, Tsize_el>& rhs) {
            if (this != &rhs){
                // Clear the current one
                clear();
                copyAll(rhs);
            }
            return *this;
        };
        
        // Move things
        /**
         * @brief Generic move constructor
         * 
         * @param other Base ElemArray to move
         */
        ElemArray(ElemArrayBase&& other) : ElemArray(other.size_el) {
            if (Tsize_el && Tsize_el != other.elem_size())
                throw std::invalid_argument("Element sizes must match to move to static ElemArray's.");
            // Move everything
            swapAll(other);
        };
        /**
         * @brief Specialized move constructor
         * 
         * @param other ElemArray to move
         */
        ElemArray(ElemArray<Tprimative_t, Tsize_el>&& other) : ElemArray() {
            swapAll(other);
        };
        /**
         * @brief Generic move assignment
         * 
         * @param rhs Base ElemArray to move
         * @return self
         */
        ElemArray<Tprimative_t, Tsize_el>& operator=(ElemArrayBase&& rhs){
            if (this != &rhs){
                if (Tsize_el && Tsize_el != rhs.elem_size())
                    throw std::invalid_argument("Element sizes must match to move to static ElemArray's.");
                // Clear the current one
                clear();
                // Move it all
                swapAll(rhs);
            }
            return *this;
        };
        /**
         * @brief Specialized move assignment
         * 
         * @param rhs ElemArray to move
         * @return self
         */
        ElemArray<Tprimative_t, Tsize_el>& operator=(ElemArray<Tprimative_t, Tsize_el>&& rhs){
            if (this != &rhs){
                // Clear the current one
                clear();
                // Move it all
                swapAll(rhs);
            }
            return *this;
        };

        // Destructor
        ~ElemArray() {
            clear();
        };

        //
        //==============================================//
        //           ITERATORS AND ACCESSORS            //
        //==============================================//
        Tprimative_t* dptr(unsigned int i = 0){
            return ((Tprimative_t*)arr) + i;
        }
        // Direct coefficient access without wraparound (eigen intercompat for nanoflann)
        Tprimative_t& coeff(unsigned int row, unsigned int col){
            return *dptr(rowIdx(row)+col);
        }
        Tprimative_t coeff(unsigned int row, unsigned int col) const{
            return coeff(row,col);
        }
        // Access row as Eigen col vector, with wraparound
        Eigen::Map<EigenRowVec_t> getVecElem(int idx){
            int a = ((idx % int(len)) + len) % len;
            return Eigen::Map<EigenRowVec_t>(dptr(rowIdx(a)), elem_size());
        };
        // Access col as Eigen col vector, with wraparound
        Eigen::Map<EigenColVec_t, Eigen::AlignmentType::Unaligned, EigenInnerStride_t > getVecIdx(int idx){
            int a = ((idx % int(elem_size())) + elem_size()) % elem_size();
            return Eigen::Map<EigenColVec_t, Eigen::AlignmentType::Unaligned, EigenInnerStride_t >(dptr(a), len, EigenInnerStride_t(elem_size()));
        };

        // Access data as Eigen Matrix, each row is the element
        Eigen::Map<EigenMatrix_t, Eigen::AlignmentType::Aligned16> getEigenMatrix(){
            return Eigen::Map<EigenMatrix_t, Eigen::AlignmentType::Aligned16 >(dptr(0), len, elem_size());
        };
        Eigen::Map<EigenArray_t, Eigen::AlignmentType::Aligned16> getEigenArray(){
            return Eigen::Map<EigenArray_t, Eigen::AlignmentType::Aligned16 >(dptr(0), len, elem_size());
        };
        // Access row as Elem, with wraparound
        // contains pointer to same memory space, do not store as reference!
        Elem operator[](int idx){
            int a = ((idx % int(len)) + len) % len;
            Elem ret(elem_size());
            ret._elem = (void*)dptr(rowIdx(a));
            return ret;
        };
        // Iterators
        Iterator begin() { return Iterator(dptr(0), elem_size()); };
        Iterator end() { return Iterator(dptr(rowIdx(len)), elem_size()); };

        //==============================================//
        //     OPERATIONS THAT MANIPULATE THE DATA      //
        //==============================================//
        /**
         * @brief Appends to the end of the ElemArray based on length for a generic ElemArray.
         * 
         * @param toAdd ElemArrayBase to append
         * @return self for chaining
         */
        ElemArray<Tprimative_t, Tsize_el>& append(const ElemArrayBase& toAdd) {
            if (toAdd.elem_size() != elem_size())
                throw std::invalid_argument("Element sizes must match to append to ElemArray.");
            
            //Append the data
            copyAppendArr(toAdd.getRawMemory(), toAdd.len);

            // Return this in case we want to do some chaining.
            return *this;
        };
        /**
         * @brief Appends to the end of the ElemArray based on length for the matching ElemArray.
         * 
         * @param toAdd ElemArrayBase to append
         * @return self for chaining
         */
        ElemArray<Tprimative_t, Tsize_el>& append(const ElemArray<Tprimative_t, Tsize_el>& toAdd) {
            return Helper::append(this, toAdd);
        };

        /**
         * @brief Trims the array to the set indices, with wraparound support.
         * 
         * @param start Start index, included
         * @param end End index, not included
         * @return self for chaining
         */
        ElemArray<Tprimative_t, Tsize_el>& trim(int start, int end) {
            // find the wraparound indices, then move with memmove
            int a = ((start % int(len)) + len) % len;
            len = (((end - 1) % int(len)) + len) % len - a + 1;
            std::memmove(dptr(), dptr(rowIdx(a)), len*_sizeof());
            return *this;
        };

        /**
         * @brief Generates a comparison function for a specific index
         * of an element, with wraparound support.
         * 
         * @param idx Index to be compared
         * @return ElemArrayBase::ElemBase::CompareFcn_t 
         */
        ElemArrayBase::ElemBase::CompareFcn_t compareIdx(int idx) {
            // compute the index and capture that
            idx = ((idx % int(elem_size())) + elem_size()) % elem_size();
            // Then return the direct access lambda function
            return [idx](const ElemBase& a, const ElemBase& b) -> bool
                    { return ((Tprimative_t*)a._elem)[idx] < ((Tprimative_t*)b._elem)[idx]; };
        };

        /**
         * @brief Performs a std::sort on a specific index. Wrapper function
         * that enables chaining and performs index wraparound.
         * 
         * @param idx Index to sort across
         * @return self for chaining
         */
        ElemArray<Tprimative_t, Tsize_el>& sortIdx(int idx) {
            std::sort(begin(), end(), compareIdx(idx));
            return *this;
        };

        /**
         * @brief Performs a std::shuffle in place using the provided
         * mt19937 generator. Does not have a built in one.
         * 
         * @param gen mt19937 random number generator.
         * @return self for chaining
         */
        ElemArray<Tprimative_t, Tsize_el>& shuffle(std::mt19937& gen) {
            std::shuffle(begin(), end(), gen);
            return *this;
        };

        /**
         * @brief Creates a clone of this ElemArray. Cloned version is
         * dynamically allocated and it's memory must be freed with the delete keyword.
         * 
         * @return ElemArray<Tprimative_t, Tsize_el>* 
         */
        ElemArray<Tprimative_t, Tsize_el>* clone() const {
            return new ElemArray<Tprimative_t, Tsize_el>(*this);
        };

        /**
         * @brief Empties the ElemArray by setting it to zero length.
         * Does not free memory or shrink the real size.
         * 
         * @return self for chaining
         */
        ElemArray<Tprimative_t, Tsize_el>& setEmpty() {
            len = 0;
            return *this;
        };

        /**
         * @brief Reserves space of length without initializing any of it.
         * Updates the length of the ElemArray if expand_length is true.
         * Does not initialize any of the space reserved!
         * 
         * @param toReserve # of elem's to expand to
         * @param expand_length Whether or not to update the displayed length
         * @return self for chaining
         */
        ElemArray<Tprimative_t, Tsize_el>& reserve(std::size_t toReserve, bool expand_length = false) {
            // Create the memory needed, if needed
            if (!arr){
                allocArr(toReserve);
                length_real = toReserve;
            }

            // If there isn't enough space at the end to add, then just get space
            else if (length_real < toReserve) {
                // Use realloc to try to expand the same space of memory.
                reallocArr(toReserve);
                length_real = toReserve;
            }

            // Make sure the allocations worked
            if (!arr) throw std::bad_alloc();

            // Then update the length if requested
            if (expand_length)
                len = toReserve;
            // Return self for chaining
            return *this;
        };
        
        /**
         * @brief Construct a new ElemArray object based on the Eigen Matrix / Array
         * 
         * @param mat
         *      The EigenBase object to copy from.
         * 
         */
        template <typename Derived>
        ElemArray(const Eigen::EigenBase<Derived>& mat)
                  : ElemArrayBase(Tsize_el), arr(nullptr)
        {
            int cols = mat.cols();
            int rows = mat.rows();
            // If this is the static case, check that the cols match, or that it's size 1 with matching rows
            if (Tsize_el){
                if (cols == Tsize_el){
                    reserve(rows, true);
                }else if(cols == 1 && rows == Tsize_el){
                    reserve(1, true);
                }else{
                    throw std::invalid_argument("Eigen cols must match size or must be 1 with matching length cols to copy to static ElemArray's.");
                }
            }
            // If this is the dynamic case, check if cols is size 1. if it is, then size_el is the rows
            else{
                if (cols == 1){
                    size_el = rows;
                    reserve(1, true);
                } else{
                    size_el = cols;
                    reserve(rows, true);
                }
            }
            // Copy it (this means sparse matrices also work)
            // Force dynamic
            Eigen::Map<Eigen::Array<Tprimative_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>, Eigen::AlignmentType::Aligned16>
                eigenArr(dptr(0), len, elem_size());
            // Account for column vector case.
            if(cols == 1){
                eigenArr = mat.derived().transpose();
            }else{
                eigenArr = mat.derived();
            }
        };

        /**
         * @brief Appends Eigen to end of ElemArray
         * 
         * @param toAdd Eigen thingamabobber to append
         * @return self for chaining
         */
        template <typename Derived>
        ElemArray<Tprimative_t, Tsize_el>& append(const Eigen::EigenBase<Derived>& toAdd) {
            int cols = toAdd.cols();
            int rows = toAdd.rows();
            auto len = length();
            // check that the cols match, or that it's size 1 with matching rows
            if (cols == elem_size()){
                reserve(len + rows, true);
            }else if(cols == 1 && rows == elem_size()){
                reserve(len + 1, true);
            }else{
                throw std::invalid_argument("Eigen cols must match size or must be 1 with matching length cols to copy to static ElemArray's.");
            }
            // Copy it (this means sparse matrices also work)
            // Force dynamic (also works for column case)
            Eigen::Map<Eigen::Array<Tprimative_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>, Eigen::AlignmentType::Unaligned>
                eigenArr(dptr(rowIdx(len)), rows, cols);
            eigenArr = toAdd.derived();

            return *this;
        };
        
    };

    //===================================//
    //         Stupid Templates          //
    //      Special Helper Classes       //
    //===================================//

    template <typename ElemArray_t, typename Tprimative_t, std::size_t Tsize_el>
    class ElemArrayHelper {
        public:
        // Static version for _sizeof()
        static unsigned int _sizeof(const ElemArray_t *parent)
        { return sizeof(Tprimative_t)*Tsize_el; };

        // Static version of elem_size()
        static std::size_t elem_size(const ElemArray_t *parent)
        { return Tsize_el; };

        // Static version of rowIdx()
        static unsigned int rowIdx(const ElemArray_t *parent, unsigned int idx)
        { return Tsize_el*idx; };

        // Static version of append
        static ElemArray_t& append(ElemArray_t *parent, const ElemArray_t& toAdd) {
            parent->copyAppendArr(toAdd.arr, toAdd.len);
            return *parent;
        }
    };

    template <typename ElemArray_t, typename Tprimative_t>
    class ElemArrayHelper<ElemArray_t, Tprimative_t, 0> {
        public:
        // Dynamic version for _sizeof()
        static unsigned int _sizeof(const ElemArray_t *parent)
        { return sizeof(Tprimative_t)*(parent->size_el); };

        // Dynamic version of elem_size()
        static std::size_t elem_size(const ElemArray_t *parent)
        { return (parent->size_el); };

        // Dynamic version of rowIdx()
        static unsigned int rowIdx(const ElemArray_t *parent, unsigned int idx)
        { return (parent->size_el)*idx; };

        // Dynamic version of append
        static ElemArray_t& append(ElemArray_t *parent, const ElemArray_t& toAdd) {
            if (toAdd.elem_size() != parent->size_el)
                throw std::invalid_argument("Element sizes must match to append to ElemArray.");
            parent->copyAppendArr(toAdd.arr, toAdd.len);
            return *parent;
        }
    };
};

//Swap
namespace std{
    template <typename Tprimative_t, std::size_t Tsize_el>
    void swap(typename sel_map::core::ElemArray<Tprimative_t, Tsize_el>::Elem& lhs, typename sel_map::core::ElemArray<Tprimative_t, Tsize_el>::Elem& rhs) {
        lhs.swap(rhs);
    };
    template <typename Tprimative_t, std::size_t Tsize_el>
    void swap(typename sel_map::core::ElemArray<Tprimative_t, Tsize_el>::Iterator& lhs, typename sel_map::core::ElemArray<Tprimative_t, Tsize_el>::Iterator& rhs) {
        lhs.swap(rhs);
    };
};
// Specific typedef for the dynamic ElemArrays
//template <typename Tprimative_t>
//using sel_map::core::ElemArrayDyn = sel_map::core::ElemArray<Tprimative_t, 0>;