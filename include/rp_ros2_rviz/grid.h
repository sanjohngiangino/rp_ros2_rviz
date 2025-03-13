#pragma once
#include <cstdint> // for uint8_t
#include <utility> // for std::pair
#include <Eigen/Core>

using Vector2i=Eigen::Matrix<int,2,1>;

struct Grid {
  using CellType=uint8_t;
  int rows=0;
  int cols=0;
  CellType* values=0;

  void clear();
  void resize(int r, int c);

  Grid(int rows=0, int cols=0);
  ~Grid();

  // disabling assignment and copy ctor
  Grid& operator = (const Grid&) = delete;
  Grid(const Grid&) = delete;

  inline CellType& at(int r, int c){
    return values[r*cols+c];
  }

  inline const CellType& at(int r, int c) const{
    return values[r*cols+c];
  }

  inline bool inside(int r, int c) const {
    return r>=0 && r<rows && c>=0 && c<cols;
  }

  inline std::pair<int, int> ptr2rc(const CellType* ptr) const {
    int offset=ptr-values;
    return std::pair<int, int>(offset/cols, offset%cols);
  }

  int scanSegment(int& x,
                  int& y,
                  float angle,
                  const CellType& val_min,
                  const int max_range) const;

  int getValue(int r, int c) const {
    if (r >= 0 && r < rows && c >= 0 && c < cols) {
      return values[r * cols + c];  // Usa l'operatore [] per l'accesso all'array
      } else {
        return -1;  // Gestisci i casi fuori dai limiti
    }
}

};
