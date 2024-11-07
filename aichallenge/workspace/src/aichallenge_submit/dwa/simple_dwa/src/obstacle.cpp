#include "obstacle.hpp"

Obstacle::Obstacle(double x, double y, double size)
    : x_(x), y_(y), size_(size) {}

double Obstacle::getX() const
{
  return x_;
}

double Obstacle::getY() const
{
  return y_;
}

double Obstacle::getSize() const
{
  return size_;
}