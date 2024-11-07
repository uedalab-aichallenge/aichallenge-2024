#pragma once

class Obstacle
{
public:
  Obstacle(double x, double y, double size);

  double getX() const;
  double getY() const;
  double getSize() const;

private:
  double x_;
  double y_;
  double size_;
};
