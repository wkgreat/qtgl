#include "../texture.hpp"

int main() {
  std::string px_path = "E:\\codes\\practice\\qtgl\\data\\box_zoom\\pos-x.jpg";
  std::string py_path = "E:\\codes\\practice\\qtgl\\data\\box_zoom\\pos-y.jpg";
  std::string pz_path = "E:\\codes\\practice\\qtgl\\data\\box_zoom\\pos-z.jpg";
  std::string nx_path = "E:\\codes\\practice\\qtgl\\data\\box_zoom\\neg-x.jpg";
  std::string ny_path = "E:\\codes\\practice\\qtgl\\data\\box_zoom\\neg-y.jpg";
  std::string nz_path = "E:\\codes\\practice\\qtgl\\data\\box_zoom\\neg-z.jpg";
  qtgl::CubeTexture cubeTexture(px_path, py_path, pz_path, nx_path, ny_path, nz_path);

  Eigen::Vector3d d = {0.392278, -0.207144, -0.89622};
  d.normalize();
  qtgl::Color01 color = cubeTexture.sample(d);
  std::cout << "Color: " << color[0] << "," << color[1] << "," << color[2] << "," << color[3]
            << std::endl;

  return 0;
}