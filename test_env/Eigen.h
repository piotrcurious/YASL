#ifndef EIGEN_MOCK_H
#define EIGEN_MOCK_H

#include <vector>
#include <cmath>
#include <iostream>
#include <numeric>

struct ArrayProxy {
    ArrayProxy() {}
    ArrayProxy square() { return *this; }
    double sum() { return 0.0; }
    double abs_sum() { return 0.0; }

    // Support Array - double
    ArrayProxy operator-(double v) const { return *this; }
};

struct VectorXd {
    std::vector<double> data;
    VectorXd(int size = 0) : data(size, 0.0) {}
    int size() const { return (int)data.size(); }
    double mean() const {
        if (data.empty()) return 0.0;
        return std::accumulate(data.begin(), data.end(), 0.0) / data.size();
    }
    ArrayProxy array() const { return ArrayProxy(); }
    double& operator()(int i) { return data[i]; }
    double operator()(int i) const { return data[i]; }

    VectorXd operator-(double v) const { return *this; }
    VectorXd operator/(double v) const { return *this; }

    void operator=(const ArrayProxy& p) {}
};

struct BlockProxy {
    double sum() { return 0.0; }
    void operator=(const VectorXd& v) {}
};

struct MatrixXd {
    int r, c;
    MatrixXd(int rows = 0, int cols = 0) : r(rows), c(cols) {}
    int rows() const { return r; }
    int cols() const { return c; }
    VectorXd row(int i) const { return VectorXd(c); }
    VectorXd col(int i) const { return VectorXd(r); }
    BlockProxy block(int r, int c, int rs, int cs) { return BlockProxy(); }
    BlockProxy block(int r, int c, int rs, int cs) const { return BlockProxy(); }
};

struct PCA {
    void computePCA(const MatrixXd& m) {}
    MatrixXd transform(const MatrixXd& m) { return m; }
};

struct ML {
    void trainEnsembleModels(const MatrixXd& f, const VectorXd& l) {}
    double predictPresence(const MatrixXd& f) { return 0.5; }
};

#endif
