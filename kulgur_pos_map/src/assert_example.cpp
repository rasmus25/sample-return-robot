#include <mrpt/base.h>         // Include all classes in mrpt-base and its dependencies
#include <mrpt/gui.h>

using namespace mrpt;          // Global methods, and data types.
using namespace mrpt::utils;   // Select namespace for serialization, utilities, etc...
using namespace mrpt::poses;   // Select namespace for 2D & 3D geometry classes.
using namespace mrpt::gui;

CDisplayWindowPlots mapPlot("Particles");
CImage 				mapImage;

int main(int argc, char **argv)
{
	mapPlot.plot(vector<double>(1, 0), vector<double>(1, 0), "o10");
	mapPlot.axis(0, 100, 0, 100);	// Tried also the size of the image in pixls

	mapImage.loadFromFile("map.bmp");
	mapPlot.image(mapImage, 0, 0, 1495, 781);

	mapPlot.axis_fit();
	mapPlot.waitForKey();

	return 0;
}