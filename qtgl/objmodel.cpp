#include "objmodel.hpp"

namespace qtgl {

ObjMaterialLib* ObjMaterialLib::loadMtlLib(std::string& dirpath, std::string& libname) {
  std::string mtlpath = dirpath + "/" + libname;
  std::ifstream ifs;
  ifs.open(mtlpath, std::ios::in);
  if (!ifs.is_open()) {
    return nullptr;
  }
  ObjMaterialLib* mtllib = new ObjMaterialLib(dirpath, libname);
  std::string line;
  std::string mtlname = "";
  int i = 0;

  while (getline(ifs, line)) {
    if (i++ % 10 == 0) {
      std::cout << "loadMtlLib: " << i << std::endl;
    }
    QString qline = QString::fromStdString(line);
    if (qline.startsWith("newmtl")) {
      QStringList lst = qline.split(QRegExp("\\s+"));
      mtlname = lst[1].toStdString();
      mtllib->mtls[mtlname] = ObjMaterial(dirpath, mtlname);
    } else if (qline.startsWith("Ns")) {
      QStringList lst = qline.split(QRegExp("\\s+"));
      mtllib->mtls[mtlname].ns = lst[1].toDouble();
    } else if (qline.startsWith("Ka")) {
      QStringList lst = qline.split(QRegExp("\\s+"));
      mtllib->mtls[mtlname].ka = {lst[1].toDouble(), lst[2].toDouble(), lst[3].toDouble(), 1};
    } else if (qline.startsWith("Kd")) {
      QStringList lst = qline.split(QRegExp("\\s+"));
      mtllib->mtls[mtlname].kd = {lst[1].toDouble(), lst[2].toDouble(), lst[3].toDouble(), 1};
    } else if (qline.startsWith("Ks")) {
      QStringList lst = qline.split(QRegExp("\\s+"));
      mtllib->mtls[mtlname].ks = {lst[1].toDouble(), lst[2].toDouble(), lst[3].toDouble(), 1};
    } else if (qline.startsWith("Ke")) {
      QStringList lst = qline.split(QRegExp("\\s+"));
      mtllib->mtls[mtlname].ke = {lst[1].toDouble(), lst[2].toDouble(), lst[3].toDouble(), 1};
    } else if (qline.startsWith("Ns")) {
      QStringList lst = qline.split(QRegExp("\\s+"));
      mtllib->mtls[mtlname].ns = lst[1].toDouble();
    } else if (qline.startsWith("Ni")) {
      QStringList lst = qline.split(QRegExp("\\s+"));
      mtllib->mtls[mtlname].ni = lst[1].toDouble();
    } else if (qline.startsWith("d")) {
      QStringList lst = qline.split(QRegExp("\\s+"));
      mtllib->mtls[mtlname].d = lst[1].toDouble();
    } else if (qline.startsWith("illum")) {
      QStringList lst = qline.split(QRegExp("\\s+"));
      mtllib->mtls[mtlname].illum = lst[1].toInt();
    } else if (qline.startsWith("map_refl")) {
      QStringList lst = qline.split(QRegExp("\\s+"));
      mtllib->mtls[mtlname].map_refl = lst[1].toStdString();
    } else if (qline.startsWith("map_Ka")) {
      QStringList lst = qline.split(QRegExp("\\s+"));
      mtllib->mtls[mtlname].map_ka = lst[1].toStdString();
    } else if (qline.startsWith("map_Kd")) {
      QStringList lst = qline.split(QRegExp("\\s+"));
      mtllib->mtls[mtlname].map_kd = lst[1].toStdString();
    }
  }
  ifs.close();
  return mtllib;
}

const std::string ObjModel::defaultGroup = "default";

ObjModel* ObjModel::loadObj(const std::string& objpath) {
  std::string group = ObjModel::defaultGroup;
  std::string mtl = "";
  std::ifstream ifs;
  ifs.open(objpath, std::ios::in);

  if (!ifs.is_open()) {
    return nullptr;
  }
  ObjModel* model = new ObjModel;
  model->objpath = objpath;
  model->dirpath = objpath.substr(0, objpath.find_last_of("/\\"));
  model->objname = objpath.substr(objpath.find_last_of("/\\") + 1);
  std::string line;
  int i = 0;

  std::list<double> verticeLst;
  std::list<double> normalLst;
  std::list<double> texcoordLst;

  while (getline(ifs, line)) {
    if (i++ % 1000 == 0) {
      std::cout << "loadObj: " << i << std::endl;
    }
    QString qline = QString::fromStdString(line);
    if (qline.startsWith("v ")) {  // vertex
      QStringList strlst = qline.split(QRegExp("\\s+"));

      // model->pushVertice(strlst[1].toDouble(), strlst[2].toDouble(), strlst[3].toDouble());
      verticeLst.push_back(strlst[1].toDouble());
      verticeLst.push_back(strlst[2].toDouble());
      verticeLst.push_back(strlst[3].toDouble());

    } else if (qline.startsWith("f ")) {  // face

      QStringList strlst = qline.split(QRegExp("\\s+"));
      std::vector<int> vi;
      std::vector<int> ni;
      std::vector<int> ti;
      for (int i = 1; i < strlst.length(); ++i) {
        QStringList lst = strlst[i].split("/");
        vi.push_back(lst[0].toInt() - 1);
        if (lst.length() >= 2 && !lst[1].isEmpty()) {
          ti.push_back(lst[1].toInt() - 1);
        } else {
          ti.push_back(-1);
        }
        if (lst.length() >= 2 && !lst[2].isEmpty()) {
          ni.push_back(lst[2].toInt() - 1);
        }
      }
      Index3 idx{vi[0], vi[1], vi[2]};
      model->addIndex3(group, idx);  // TODO optimize

      if (!ti.empty()) {
        Eigen::Vector3i texidx{ti[0], ti[1], ti[2]};
        model->addTexRef(group, mtl, texidx);
      }

      if (!ni.empty()) {
        NormIndex normidx{ni[0], ni[1], ni[2]};
        model->addNormIndex(group, normidx);  // TODO optimize
      }
    } else if (qline.startsWith("g ")) {  // group
      QStringList lst = qline.trimmed().split(QRegExp("\\s+"));
      if (lst.length() < 2) {
        group = defaultGroup;
      } else {
        group = lst[1].toStdString();
      }

      std::cout << group << std::endl;

    } else if (qline.startsWith("vn")) {  // normal
      QStringList strlst = qline.split(QRegExp("\\s+"));

      // model->pushNormal(strlst[1].toDouble(), strlst[2].toDouble(), strlst[3].toDouble());
      normalLst.push_back(strlst[1].toDouble());
      normalLst.push_back(strlst[2].toDouble());
      normalLst.push_back(strlst[3].toDouble());

    } else if (qline.startsWith("vt")) {  // texture
      QStringList strlst = qline.split(QRegExp("\\s+"));

      // model->pushTexCoord(strlst[1].toDouble(), strlst[2].toDouble());
      texcoordLst.push_back(strlst[1].toDouble());
      texcoordLst.push_back(strlst[2].toDouble());

    } else if (qline.startsWith("#")) {
      continue;  // do nothing
    } else if (qline.startsWith("mtllib")) {
      model->mtllib =
          ObjMaterialLib::loadMtlLib(model->dirpath, qline.split(QRegExp("\\s+"))[1].toStdString());
    } else if (qline.startsWith("o ")) {
      // TODO
    } else if (qline.startsWith("s ")) {
      // TODO
    } else if (qline.startsWith("usemtl")) {
      QStringList lst = qline.trimmed().split(QRegExp("\\s+"));
      mtl = lst[1].toStdString();
    }
  }

  model->setVertices(verticeLst);
  model->setNormals(normalLst);
  model->setTexcoords(texcoordLst);

  ifs.close();
  return model;
}
}  // namespace qtgl