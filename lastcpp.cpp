/*
// Definition for a Node.
class Node {
public:
    int val;
    Node* prev;
    Node* next;
    Node* child;
};
*/

class Solution {
private:
    Node* merge(Node* list1 , Node* list2){
    Node* dummy = new Node(-1);
    Node* linker = dummy;
    while(list1!=nullptr && list2!=nullptr){
        if(list1->val <= list2->val){
            linker -> child = list1;
            linker = linker -> child;
            list1 = list1 -> child;
        }
        else{
            linker -> child = list2;
            linker = linker -> child;
            list2 = list2 -> child;
        }
        linker -> next = nullptr;
    }
    if(list1 != nullptr){
        linker -> child = list1;
    }
    else{
        linker -> child = list2;
    }
    return dummy->child;
}
public:
    Node* flatten(Node* head) {
        if(head == nullptr || head -> next == nullptr) return head;
        head -> next = flatten(head -> next);
        head = merge(head,head->next);
        return head;
    }
};