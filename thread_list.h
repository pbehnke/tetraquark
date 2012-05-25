// thread_t is a pointer to function with no parameters and
// no return value...i.e., a user-space thread.
typedef void (*thread_t)(void);

thread_node *threadList;

typedef struct nodeStructureType {
	thread_t thread_func; //pointer to thread function
	int active;           // non-zero means thread is allowed to run
  	char *stack;          // pointer to TOP of stack (highest memory location)
  	unsigned state[40];   //space to store important registers
	struct nodeStructureType *next; //pointer to next object in linked list
} thread_node;
