#include "rp_ros2_rviz/grid_map.h"
#include <cstring>

GridMap::GridMap(int rows,
                 int cols,
                 float res,
                 WorldItem* p,
                 const Isometry2f& pose_in_parent_):
  Grid(rows, cols),
  WorldItem(p),
  resolution(res),
  inv_resolution(1./res),
  grid_origin(rows/2, cols/2),
  _piw(poseInWorld()),
  _ipiw(poseInWorld().inverse())
{
  pose_in_parent=pose_in_parent_;
}

bool GridMap::canCollide(const WorldItem* other) const {
  bool ret=other->isDescendant(this);
  //cerr << "gmap " << this << " can collide " << other << endl;
  return ret;
}

bool GridMap::collides(const WorldItem* item) const {
  if (! item->isDescendant(this))
    return false;
  Vec2f gp=w2g(item->poseInWorld().t);
  int r0=gp.x();
  int c0=gp.y();
  //cerr << "r0: " << r0 << " c0:" << c0 << endl;
  int radius=(int)(item->radius*inv_resolution);
  int radius2=radius*radius;
  for (int r=-radius; r<radius; ++r){
    int rx=r+r0;
    for (int c=-radius; c<radius; ++c) {
      if (c*c+r*r>radius2)
        continue;
      int cx=c+c0;
      if (! inside(rx,cx))
        continue;
      if (at(rx,cx)<127)
        return true;
    }
  }
  return false;
}

// ctor that loads from image
GridMap::GridMap(const char* filename,
                 float res,
                 WorldItem* p,
                 const Isometry2f& pose_in_parent_):
  WorldItem(p)
{
  pose_in_parent=pose_in_parent_;
  resolution=res;
  //load using CV
  cv::Mat m=cv::imread(filename);
  if (m.rows==0) {
    throw std::runtime_error("unable to load image");
  }
  cv::cvtColor(m, cv_image, cv::COLOR_BGR2GRAY);
  int size=cv_image.rows*cv_image.cols;
  resize(cv_image.rows, cv_image.cols);
  resolution=res;
  inv_resolution=1./res;
  grid_origin=Vec2f(rows/2, cols/2);
  _piw=poseInWorld();
  _ipiw=_piw.inverse();
  memcpy(values, cv_image.data, size);
}
  
void GridMap::draw(Canvas& dest) const {
  Rotation2f Rt=_piw.R.inverse();
  Rotation2f sRt=Rt.scale(inv_resolution);
  Rotation2f s2Rt=Rt.scale(dest.resolution*inv_resolution);
  Vec2f sT=grid_origin-sRt*(dest.canvas_origin*dest.resolution+_piw.t);
  Vec2f t=_piw.t*(1./dest.resolution);
  for (int r=0; r<dest.rows(); ++r)
    for (int c=0; c<dest.cols(); ++c){
      Vec2f dest_v(r,c);
      Vec2f src_v=s2Rt*Vec2f(r,c)+sT;
      int src_r=src_v.x();
      int src_c=src_v.y();
      if (inside(src_r, src_c))
        dest.draw_image.at<uint8_t>(r,c)=cv_image.at<uint8_t>(src_r, src_c);
    }
  WorldItem::draw(dest);
}

void GridMap::printGridMap() const {
  for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
          int value = getValue(r, c);  // O usa cv_image per ottenere i valori
          std::cout << value << " ";
      }
      std::cout << std::endl;
  }
}

Vector2iVector GridMap::getObstacles() const {
  Vector2iVector obstacles;  

  // Itera su tutte le celle della griglia
  for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
          // Ottieni il valore della cella dalla griglia
          int value = getValue(r, c);  // Usa getValue per ottenere il valore della cella
          // Considera gli ostacoli se il valore della cella è maggiore di una soglia
          if (value < 127) {  // Cambia la soglia a seconda dei tuoi dati
              obstacles.push_back(Vector2i(c, r)); // Aggiungi le coordinate dell'ostacolo
          }
      }
  }
  
  return obstacles;
}


void GridMap::drawObstacles(Canvas& dest) const {
  Vector2iVector obstacles = getObstacles();

  Rotation2f Rt = _piw.R.inverse();                      // Rotazione inversa
  Rotation2f sRt = Rt.scale(inv_resolution);              // Scala per la risoluzione della grid
  Rotation2f s2Rt = Rt.scale(dest.resolution * inv_resolution*0.25);  // Scala per la risoluzione della canvas

  dest.draw_image.setTo(255);

  Vec2f sT = grid_origin - sRt * (dest.canvas_origin * dest.resolution + _piw.t);

  for (const auto& obstacle : obstacles) {
    int r = obstacle.x();  
    int c = obstacle.y(); 

    Vec2f dest_v(c, r); 
    Vec2f src_v = s2Rt * dest_v + sT;
    int src_r = src_v.x();
    int src_c = src_v.y();

    if (src_r >= 0 && src_r < dest.rows() && src_c >= 0 && src_c < dest.cols()) {
      dest.draw_image.at<uint8_t>(src_r, src_c) = 0;  // 0 potrebbe essere il valore per il "nero" per un ostacolo visibile
    }
  }
  WorldItem::draw(dest);
}

