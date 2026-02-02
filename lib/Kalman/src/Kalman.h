#pragma once
#include "AllSensors.h"
#include <BasicLinearAlgebra.h>

class Data {
    public:
        Data();

        void initializeMatrices();

        void predict();
        void update();
};
