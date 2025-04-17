#include "shader.hpp"

#include "shadow.hpp"

namespace qtgl {

void GLTriangleShader::shade(Triangle3& t, GLMaterial* m, GLCamera& camera,
                             std::vector<GLLight*>& lights, std::vector<GLShadowMapping*>& shadows,
                             Color01 ambient, Fragments& fragments,
                             Eigen::Matrix4d& invTransformMatrix) {
  // 背面剔除
  if (isTriangleBack(camera, t)) {
    return;
  }

  // mbr
  int xmin = static_cast<int>(std::min(std::min(t.hx0(), t.hx1()), t.hx2()));
  int xmax = static_cast<int>(std::max(std::max(t.hx0(), t.hx1()), t.hx2()));
  int ymin = static_cast<int>(std::min(std::min(t.hy0(), t.hy1()), t.hy2()));
  int ymax = static_cast<int>(std::max(std::max(t.hy0(), t.hy1()), t.hy2()));

  double depth;
  Color01 color;
  Triangle3::BarycentricCoordnates coord;
  IlluminationModel model = m->getIllumination();
  std::vector<int> isLightShadow(lights.size());
  GLShader* shader = getShader(m->getIllumination());

  // clip TODO

  for (int x = xmin; x <= xmax; ++x) {
    if (x < 0 || x >= fragments[0].size()) continue;
    for (int y = ymin; y <= ymax; ++y) {
      if (y < 0 || y >= fragments.size()) continue;

      // Fragment shader

      // Barycentric test
      coord = t.resovleBarycentricCoordnates({x, y});
      if (coord.alpha < 0 || coord.beta < 0 || coord.gamma < 0) {
        continue;
      }

      // depth test
      depth = coord.alpha * t.hz0() + coord.beta * t.hz1() + coord.gamma * t.hz2();
      if (depth >= fragments[y][x].depth) {
        continue;
      }

      if (depth < fragments[y][x].depth) {
        // shadow
        Vertice screenPos(x, y, depth, 1);
        Vertice worldPos = verticeTransform(invTransformMatrix, screenPos);
        for (int i = 0; i < lights.size(); ++i) {
          if (shadows[i]->isWorldPosInShadow(worldPos)) {
            isLightShadow[i] = 1;
          } else {
            isLightShadow[i] = 0;
          }
        }

        if (model == IlluminationModel::CONSTANT) {
          color = m->getDiffuse();
        } else {
          color = shader->shade2(t, worldPos, camera, coord, lights, isLightShadow, m, ambient);
        }

        fragments[y][x].color = color;
        fragments[y][x].depth = depth;
      }
    }
  }

  if (shader) {
    delete shader;
  }
}

}  // namespace qtgl