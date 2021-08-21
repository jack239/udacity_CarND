#include "classifier.h"
#include <math.h>
#include <string>
#include <vector>

// Initializes GNB
namespace {
TrainPoint convert(const std::vector<double>& rawPoint) {
    assert(rawPoint.size() == 4);
    return {rawPoint[0], rawPoint[1], rawPoint[3]};
}

}
GNB::GNB()
    : possible_labels{"left","keep","right"},
      trained_{false}
{
}

GNB::~GNB() {}

void GNB::train(
    const std::vector<std::vector<double>>& data,
    const std::vector<std::string>& labels
    ) {
    assert(!trained_);
    std::array<double, countTargets_> counts;

    for (size_t j = 0; j < countTargets_; ++j) {
        avers_[j].setZero();
        sds_[j].setZero();
        counts[j] = 0;
        priors_[j] = 0;
    }
    auto train = [](TrainPoint& total, double& count, const TrainPoint& point) {
        total += point;
        count += 1;
    };

    assert(data.size() == labels.size());
    for (size_t i = 0; i < data.size(); ++i) {
        const auto point = convert(data.at(i));
        for (size_t  j = 0; j < countTargets_; ++j) {
            if (labels[i] == possible_labels[j]) {
                train(avers_[j], counts[j], point);
                break;
            }
        }
    }
    for (size_t  j = 0; j < countTargets_; ++j) {
        assert(counts[j] > 0);
        avers_[j] = avers_[j] / counts[j];
        priors_[j] = counts[j] / labels.size();
    }

    auto update_sds = [](TrainPoint& sds, const TrainPoint& aver, const TrainPoint& point) {
        TrainPoint off = point - aver;
        for (size_t k = 0; k < sds.size(); ++k) {
            sds[k] += off[k] * off[k];
        }
    };

    for (size_t i = 0; i < data.size(); ++i) {
        const auto point = convert(data.at(i));
        for (size_t  j = 0; j < countTargets_; ++j) {
            if (labels[i] == possible_labels[j]) {
                update_sds(sds_[j], avers_[j], point);
                break;
            }
        }
    }

    for (size_t  j = 0; j < countTargets_; ++j) {
        for (size_t k = 0; k < sds_[j].size(); ++k) {
            sds_[j][k] = std::sqrt(sds_[j][k] / counts[j]);
        }
    }

    trained_ = true;
}

std::string GNB::predict(const std::vector<double>& sample) const {
    const auto point = convert(sample);
    std::array<double, countTargets_> probs;
    probs = priors_;

    auto square = [](double x) { return x * x; };

    for (size_t j = 0; j < countTargets_; ++j) {
        const auto& sds = sds_[j];
        const auto& aver = avers_[j];
        auto& p = probs[j];
        for (size_t k = 0; k < point.size(); ++k) {
            double exponent = - 0.5 * square(point[k] - aver[k]) / square(sds[k]);
            p *= std::exp(exponent) / std::sqrt( 2 * M_PI * square(sds[k]));
        }
    }

    auto maxIt = std::max_element(probs.begin(), probs.end());
    return possible_labels.at(maxIt - probs.begin());
    /**
     * Once trained, this method is called and expected to return
     *   a predicted behavior for the given observation.
     * @param observation - a 4 tuple with s, d, s_dot, d_dot.
     *   - Example: [3.5, 0.1, 8.5, -0.2]
     * @output A label representing the best guess of the classifier. Can
     *   be one of "left", "keep" or "right".
     *
     * TODO: Complete this function to return your classifier's prediction
     */

}