#include <stdlib.h>
#include <thread_list.h>


thread_node *createNode(thread_t newThread){
	thread_node *temp;
	if(!(temp=malloc(sizeof(thread_node)))) return NULL;
	temp->thread_func=newThread;
	temp->next=NULL;
	return temp;
}

thread_node *insert_node(thread_node *node, thread_t thread){
	thread_node *new_node;
	new_node=createNode(thread);
	new_node->next=node->next;
	node->next=new_node;
	return new_node;

}

thread_node *insert_node_beginning(thread_node *firstnode, thread_t thread){
	thread_node *new_node;
	new_node=createNode(thread);
	new_node->next=firstnode;
	return new_node;
}

int linear_traverse(thread_node *node){
	while(node) {
		node->thread_func();
		node=node->next;
	}
	return 0;
}

void registerThread(thread_t thread){
	threadList=insert_node_beginning(threadList, thread);
}
