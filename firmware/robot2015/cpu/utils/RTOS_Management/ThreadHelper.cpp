#include "ThreadHelper.hpp"

// Helper function for defining threads
void define_thread(osThreadDef_t& t, void(*task)(void const *arg), osPriority priority, uint32_t stack_size, unsigned char *stack_pointer)
{
#ifdef CMSIS_OS_RTX
    t.pthread = task;
    t.tpriority = priority;
    t.stacksize = stack_size;

    if (stack_pointer != NULL) {
        t.stack_pointer = (uint32_t*) stack_pointer;
        //_dynamic_stack = false;
    } else {
        t.stack_pointer = (uint32_t*) new unsigned char[t.stacksize];
        if (t.stack_pointer == NULL)
            error("Error allocating the stack memory\n");
        //_dynamic_stack = true;
    }
#endif
}
