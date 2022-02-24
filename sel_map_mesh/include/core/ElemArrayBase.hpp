/**
 * @file ElemArrayBase.hpp
 * @author Adam Li (adamli@umich.edu)
 * @brief The base definition creating an abstract class for all ElemArray types.
 * @date 2021-11-11
 * 
 */
#pragma once

// Standard library parts involved in the definitions
#include <cstddef>
#include <functional>

namespace sel_map::core {
    class ElemArrayBase{
        public:
        // Extra eigen intercompat for nanoflann
        typedef typename std::size_t Index;
        constexpr static const int RowsAtCompileTime = -1;

        // The length of the array
        std::size_t len;
        
        // Actual length of the whole array in memory (no shrinking is normally done)
        // This is the same as N
        std::size_t length_real;

        // A length of a single element. Only here for the dynamic case.
        std::size_t size_el;
        
        // Base Constructor
        ElemArrayBase(std::size_t size_el = 0)
                      : len(0), length_real(0), size_el(size_el) {};

        // Base Elem object, only provides raw, untyped access to the data.
        public:
        class ElemBase{
            public:
            void* _elem;
            // Comparator function pointer, for code readability later
            typedef std::function<bool(const ElemBase&, const ElemBase&)> CompareFcn_t;
            // Extra overloads
            virtual std::size_t size() const =0;
        };

        // returns the length of this array
        std::size_t length() const { return len; };
        // Get the size of a single element
        virtual std::size_t elem_size() const =0;
        // Eigen intercompat for use with nanoflann
        std::size_t rows() const { return length(); };
        std::size_t cols() const { return elem_size(); };

        // Create pure virtual constructions for almost everything below, making this effectively an abstract class / interface
        // Utility function to clear its memory
        virtual void clear() =0;
        // Copy arbitrary ElemArray, should error out if static & not equal
        //virtual ElemArrayBase(const ElemArrayBase& other) =0;
        virtual ElemArrayBase& operator=(const ElemArrayBase& rhs) =0;
        // Move arbitrary ElemArray, should error out if static & not equal
        //virtual ElemArrayBase(const ElemArrayBase&& other) =0;
        virtual ElemArrayBase& operator=(ElemArrayBase&& rhs) =0;
        // Destructor
        virtual ~ElemArrayBase() {};
        
        // Get raw access to the memory of the ElemArray (used for cross array access w/ static to dynamic)
        virtual void*& getRawMemory() =0;
        virtual void* getRawMemory() const =0;

        // Append the arbitary ElemArray to the current one. should error out if
        // elem_size() is not equal
        virtual ElemArrayBase& append(const ElemArrayBase& toAdd) =0;
        
        //virtual ElemBase* begin() =0;
        //virtual ElemBase* end() =0;
        
        // Trims the ElemArray to the ranges specified, with wraparound
        virtual ElemArrayBase& trim(int start, int end) =0;
        // Generates a comparison function for two Elem's of the ElemArray based on
        // a specific index of the Elem
        virtual ElemBase::CompareFcn_t compareIdx(int idx) =0;
        // Wrapper to sort the array by a specific element
        virtual ElemArrayBase& sortIdx(int idx) =0;
        // Wrapper to randomize the array
        virtual ElemArrayBase& shuffle(std::mt19937& gen) =0;
        // Wrapper to generate a copy of the array which would need to be destroyed
        virtual ElemArrayBase* clone() const =0;
        
        // Reserve space for the ElemArray, update length if expand_length = true
        virtual ElemArrayBase& reserve(std::size_t toReserve, bool expand_length = false) =0;

        // Empty the elements of the ElemArray
        virtual ElemArrayBase& setEmpty() =0;
    };
};
