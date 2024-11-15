#include "display.h"

int main(int argc, char **argv)
{
	auto display = std::make_unique<Display>();
	display->Render();

	return 0;
}
