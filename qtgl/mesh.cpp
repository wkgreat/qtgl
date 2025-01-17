#include "mesh.hpp"

namespace qtgl {

void GLMeshGroup::addIndex3(Index3 idx) {
  addIndex3(idx, GLMesh::defaultColor, GLMesh::defaultColor, GLMesh::defaultColor);
}

void GLMeshGroup::rasterize(GLScene& scene) {
  int n = indices.rows();
  for (int i = 0; i < n; ++i) {
    Index3 idx = indices.row(i);
    NormIndex normIdx = normIndices.row(i);
    TexRef ref = texrefs[i];
    Vertice rp0 = parent->getVertices().row(idx[0]);
    Vertice rp1 = parent->getVertices().row(idx[1]);
    Vertice rp2 = parent->getVertices().row(idx[2]);
    Vertice p0 = parent->getTransformedVertices().row(idx[0]);
    Vertice p1 = parent->getTransformedVertices().row(idx[1]);
    Vertice p2 = parent->getTransformedVertices().row(idx[2]);
    Normal n0 = parent->getTransformedNormals().row(normIdx[0]).normalized();
    Normal n1 = parent->getTransformedNormals().row(normIdx[1]).normalized();
    Normal n2 = parent->getTransformedNormals().row(normIdx[2]).normalized();

    // 背面剔除
    if (scene.isTriangleBack(rp0, rp1, rp2, n0, n1, n2)) {
      continue;
    }

    GLMaterial* material = parent->getMaterial(ref.mtlname);

    if (ref.indices[0] != -1 && ref.indices[1] != -1 && ref.indices[2] != -1) {
      TexCoord t0 = parent->getTexCoords().row(ref.indices[0]);
      TexCoord t1 = parent->getTexCoords().row(ref.indices[1]);
      TexCoord t2 = parent->getTexCoords().row(ref.indices[2]);
      Triangle2 t(p0, p1, p2, n0, n1, n2, t0, t1, t2);
      std::vector<Color01> clrs = colors[i];
      rasterizeTriangle(scene, t, clrs, material);
    } else {
      Triangle2 t(p0, p1, p2, n0, n1, n2);
      std::vector<Color01> clrs = colors[i];
      rasterizeTriangle(scene, t, clrs, material);
    }
  }
}

void GLMeshGroup::rasterizeTriangle(GLScene& scene, Triangle2& t, std::vector<Color01>& clrs,
                                    GLMaterial* material) {
  // mbr
  int xmin = static_cast<int>(std::min(std::min(t.hx0(), t.hx1()), t.hx2()));
  int xmax = static_cast<int>(std::max(std::max(t.hx0(), t.hx1()), t.hx2()));
  int ymin = static_cast<int>(std::min(std::min(t.hy0(), t.hy1()), t.hy2()));
  int ymax = static_cast<int>(std::max(std::max(t.hy0(), t.hy1()), t.hy2()));

  double depth;
  Color01 color;
  Triangle2::BarycentricCoordnates coord;
  Fragments& fragments = scene.getFragments();
  GLTexture* texture = material->getDiffuseTexture();
  IlluminationModel model = material->getIllumination();

  for (int x = xmin; x <= xmax; ++x) {
    if (x < 0 || x >= fragments[0].size()) continue;
    for (int y = ymin; y <= ymax; ++y) {
      if (y < 0 || y >= fragments.size()) continue;
      // Shade Fragment

      coord = t.resovleBarycentricCoordnates({x, y});
      if (coord.alpha >= 0 && coord.beta >= 0 && coord.gamma >= 0) {
        depth = coord.alpha * t.hz0() + coord.beta * t.hz1() + coord.gamma * t.hz2();
        if (depth < fragments[y][x].depth) {
          // decide to shade

          if (model == IlluminationModel::CONSTANT) {
            color = material->getDiffuse();
          } else {
            GLShader* shader = scene.getShader(IlluminationModel::LAMBERTIAN_BLINN_PHONG);  // TODO
            Vertice screenPos(x, y, depth, 1);
            Vertice worldPos = scene.screenVerticeBackToWorldVertice(screenPos);
            Normal uvNormal = (coord.alpha * t.getNormal0() + coord.beta * t.getNormal1() +
                               coord.gamma * t.getNormal2())
                                  .normalized();
            Normal uvView =
                (scene.getCamera().getPositionVertice().head(3) - worldPos.head(3)).normalized();
            TexCoord txtcoord =
                GLTexture::interpolateTexCoord(t, coord.alpha, coord.beta, coord.gamma);

            color = shader->shade(scene.getLights(), scene.getAmbient(), material, worldPos,
                                  uvNormal, uvView, &txtcoord);
          }

          fragments[y][x].color = color;
          fragments[y][x].depth = depth;
        }
      }
    }
  }
}

const Color01 GLMesh::defaultColor = {1, 1, 1, 1};
const std::string GLMesh::defaultGroup = "default";

void GLMesh::rasterize(GLScene& scene) {
  for (auto g : groups) {
    (g.second)->rasterize(scene);
  }
}

GLMesh* GLMesh::fromObjModel(ObjModel* model) {
  GLMesh* mesh = new GLMesh;
  mesh->vertices = model->vertices;
  mesh->normals = model->normals;
  mesh->texcoords = model->texcoords;
  for (auto g : model->groups) {
    std::string name = g.first;
    ObjModelGroup& group = g.second;
    GLMeshGroup* meshGroup = new GLMeshGroup(mesh, name);
    meshGroup->setIndices(group.indices);
    meshGroup->setNormIndices(group.normIndices);
    meshGroup->setTexRefs(group.texrefs);
    mesh->groups[name] = meshGroup;
    Color01 white = {1, 1, 1, 1};
    std::vector<std::vector<Color01>> colors(meshGroup->getIndices().rows(), {white, white, white});
    meshGroup->setColors(colors);
  }
  if (model->mtllib) {
    for (auto mtl : model->mtllib->mtls) {
      std::string name = mtl.first;
      ObjMaterial& material = mtl.second;
      mesh->materials[name] = mtl.second.toGLMaterial();
    }
  }
  return mesh;
}

GLMesh* GLMesh::readFromObjFile(std::string fpath) {
  GLMesh* mesh = nullptr;
  ObjModel* model = ObjModel::loadObj(fpath);
  if (model) {
    mesh = fromObjModel(model);
    delete model;
  }
  return mesh;
}

}  // namespace qtgl