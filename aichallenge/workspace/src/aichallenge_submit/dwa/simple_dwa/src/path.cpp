#include "path.hpp"

Path::Path(double u_th, double u_v)
    : u_v_(u_v), u_th_(u_th) {}

void Path::setX(const std::vector<double> &x)
{
  x_ = x;
}

void Path::setY(const std::vector<double> &y)
{
  y_ = y;
}

void Path::setTh(const std::vector<double> &th)
{
  th_ = th;
}

const std::vector<double> &Path::getX() const
{
  return x_;
}

const std::vector<double> &Path::getY() const
{
  return y_;
}

const std::vector<double> &Path::getTh() const
{
  return th_;
}

double Path::getUV() const
{
  return u_v_;
}

double Path::getUTh() const
{
  return u_th_;
}