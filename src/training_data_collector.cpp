#include <print>
#include <complex>
#include <cstdlib>
#include <iostream>
#include <map>
#include <string>

#include <cnpy.h>

const int Nx = 128;
const int Ny = 64;
const int Nz = 32;

int main() {
    // set random seed so that result is reproducible (for testing)
    srand(0);
    // create random data
    std::vector<std::complex<double>> data(Nx * Ny * Nz);
    for (int i = 0; i < Nx * Ny * Nz; i++) data[i] = std::complex<double>(rand(), rand());

    // save it to file
    cnpy::npy_save("../training_data/arr1.npz", &data[0], {Nz, Ny, Nx}, "wx");
    std::println("Hello, World!");
    return 0;
}