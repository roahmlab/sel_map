# RGB Semantic Segmentation with Depth Image Projection

Within this folder, there are 5 packages. Three of them are semantic segmentation networks, pre-wrapped to work with our sel_map package. All of them will pull code from their respective git repositories and perform any necessary preparation steps to use the network as well as provide arbitrary wrapper code for usage with the rest of the sel_map project. One is a template package included to simplify wrapping of your own segmentation network of choice, and the last is the library actually used by the main sel_map package to process RGB images through a segmentation network of choice and project those class labels based on the depth image and camera intrinsics.

---

## Segmentation Networks

Among the included segmentation networks, we include wrappers for:

**Pretrained PyTorch models by CSAIL trained on the MIT ADE20K scene parsing dataset** [[1]][[2]]. These are a variety of baseline pretrained models. The original repository can be found at https://github.com/CSAILVision/semantic-segmentation-pytorch, from where we use commit `8f27c9b97d2ca7c6e05333d5766d144bf7d8c31b`, and the pretrained weights which should be added to the `ckpt/` folder can be found at http://sceneparsing.csail.mit.edu/model/pytorch. We chose the `ResNet50dilated + PPM_deepsup` model as our primary model.

**A PyTorch implementation of FastSCNN** [[3]]. The specific implementation of FastSCNN here is from https://github.com/Tramac/Fast-SCNN-pytorch. We use commit `0638517d359ae1664a27dfb2cd1780a40a06c465`.

**PyTorch-Encoding Semantic Segmentation Models by Hang Zhang** [[4]]. This wrapper pulls from https://github.com/zhanghang1989/PyTorch-Encoding, and uses commit `331ecdd5306104614cb414b16fbcd9d1a8d40e1e`. This network also includes an additional compile step, which the wrapper will perform automatically. *Note: If there is a circular import error, make sure you have installed CUDA, since this network does not work on the CPU alone.*

---

## Wrapper Template

Please see the included README in the wrapper template package. It includes instructions for how to use it.

---

## sel_map_segmentation

This package includes the camera sensor model, and colorscale classes used for segmentation. It also includes additional scripts to evaluate the speed of a given semseg network as well as to output the segmentation of an image or set of images in a folder. Generally, a `terrain_properties` file and a `semseg_config` file are both used to select a network, set the colorscale, and store the base properties. Examples of these two files can be found in the `sel_map` package under the `config/terrain_properties` and `config/semseg` folders respectively.


---

[[1]]  B. Zhou, H. Zhao, X. Puig, T. Xiao, S. Fidler, A. Barriuso, and A. Torralba, “Semantic understanding of scenes through the ade20k dataset,” *International Journal on Computer Vision*, 2018.

[[2]] B. Zhou, H. Zhao, X. Puig, S. Fidler, A. Barriuso, and A. Torralba, “Scene parsing through ade20k dataset,” in *Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition*, 2017.

[[3]] R. P. K. Poudel, S. Liwicki, and R. Cipolla, “Fast-scnn: Fast semantic segmentation network,” in *BMVC*, 2019

[[4]] H. Zhang, K. Dana, J. Shi, Z. Zhang, X. Wang, A. Tyagi, and A. Agrawal, “Context encoding for semantic segmentation,” in *The IEEE Conference on Computer Vision and Pattern Recognition (CVPR)*, June 2018.

[1]: https://arxiv.org/abs/1608.05442
[2]: http://people.csail.mit.edu/bzhou/publication/scene-parse-camera-ready.pdf
[3]: https://arxiv.org/abs/1902.04502
[4]: https://arxiv.org/pdf/1803.08904.pdf