#include "rp_ros2_rviz/dmap.h"
#include <queue>
#include <iostream>

void DMap::clear(){
  std::fill(values.begin(), values.end(), DMapCell());
}

void DMap::update(const Vector2iVector& obstacles) {
  using DMapCellQueue = std::deque<DMapCell*>;
  DMapCellQueue q;

  for (const auto& o : obstacles) {
      if (!inside(o.x(), o.y())) continue;

      DMapCell& cell = at(o.x(), o.y());
      cell.parent = &cell;
      cell.dist = 0;
      q.push_back(&cell);
  }

  while (!q.empty()) {
      DMapCell* current = q.front();
      q.pop_front();

      Vector2i current_pos = ptr2rc(current);
      Vector2i parent_pos = ptr2rc(current->parent);

      for (int dr = -1; dr <= 1; ++dr) {
          for (int dc = -1; dc <= 1; ++dc) {
              if (dr == 0 && dc == 0) continue;

              Vector2i offset(dr, dc);
              Vector2i neighbor_pos = current_pos + offset;

              if (!inside(neighbor_pos.x(), neighbor_pos.y()))
                  continue;

              DMapCell& neighbor = at(neighbor_pos.x(), neighbor_pos.y());

              int dist_to_parent = (parent_pos - neighbor_pos).squaredNorm();

              if (!neighbor.parent || dist_to_parent < neighbor.dist) {
                  neighbor.parent = current->parent;
                  neighbor.dist = dist_to_parent;
                  q.push_back(&neighbor);
              }
          }
      }
  }
}


Grid_<float> DMap::distances(float max_distance) const {
  Grid_<float> dest(rows, cols);
  for (int r=0; r<rows; ++r){
    for (int c=0; c<cols; ++c) {
      const auto& cell=at(r,c);
      dest.at(r,c) = std::min(max_distance, sqrtf(parentDistance(&cell)));
    }
  }
  return dest;
}

std::ostream& operator<<(std::ostream& os, const DMap& dmap) {
  for (int r=0; r<dmap.rows; ++r){
    for (int c=0; c<dmap.cols; ++c) {
      const auto& cell=dmap.at(r,c);
      os << sqrt(dmap.parentDistance(&cell)) << " ";
    }
    os << endl;
  }
  return os;
}
