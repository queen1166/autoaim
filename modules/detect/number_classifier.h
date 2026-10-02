
#pragma once

#include <opencv2/core.hpp>

#include <vector>

namespace autoaim::detect {

struct Armor;

class NumberClassifier {
 public:
  virtual ~NumberClassifier() = default;

  virtual void extractNumbers(const cv::Mat & src, std::vector<Armor> & armors) = 0;

  virtual void classify(std::vector<Armor> & armors) = 0;

  float threshold = 0.7f;
};

}
