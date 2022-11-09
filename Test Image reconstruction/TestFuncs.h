#pragma once

#include "RealsenseControl.h"
#include "LahgiControl.h"
#include "CodeMaskCalc.h"
#include "RadiationImage.h"

#include <random>
#include <stdio.h>
#include <open3d/visualization/utility/DrawGeometry.h>
#include <zlib.h>

class TestFuncs
{

public:
	static void TestImageRecon();
	static void TestAddingLmData();
	static void TestAddingLmDataVerification(int counts);
	static void GetFloodImage();
	static void TestZlib();
	static void LoadSystemMatrix();
};

