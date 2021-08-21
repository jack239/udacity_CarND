#ifndef CLASSIFIER_H
#define CLASSIFIER_H

#include <string>
#include <vector>
#include <Eigen/Dense>

using TrainPoint = Eigen::Vector3d;
class GNB {
public:
    /**
     * Constructor
     */
    GNB();

    /**
     * Destructor
     */
    virtual ~GNB();

    /**
     * Train classifier
     */
    void train(
        const std::vector<std::vector<double>>& data,
        const std::vector<std::string>& labels
        );

    /**
     * Predict with trained classifier
     */
    std::string predict(const std::vector<double>& sample) const;

private:
    static constexpr size_t countTargets_ = 3;
    std::array<std::string, countTargets_> possible_labels;
    std::array<TrainPoint, countTargets_> avers_;
    std::array<TrainPoint, countTargets_> sds_;
    std::array<double, countTargets_> priors_;
    bool trained_;
};

#endif  // CLASSIFIER_H