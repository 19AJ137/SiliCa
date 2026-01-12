// SiliCa - JIS X 6319-4 compatible card implementation
// Main entry point

#include "physical.h"

int main()
{
    setup();

    while (true)
    {
        loop();
    }
}
