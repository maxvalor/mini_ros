#include <iostream>
#include <boost/make_shared.hpp>
struct A {
  int data;
  int* p;
  A() { data = 0; p = new int[1]; p[0] = data; }
  A(A&& a) {
    data = a.data;
    p = a.p;
    p = nullptr;
  }
  ~A() {
    if (p != nullptr) {
      std::cout << "~" << std::endl;
      delete[] p;
    }
  }
};

void func(A* a) {
  A *b = new A(std::move(*a));
  std::cout << b->data << std::endl;
}

int main() {
  A *a = new A();
  //func(a);

  const std::shared_ptr<const A> sa(a);
  //sa->data = 2;
  //delete[] a;
  std::cout << sa->data << std::endl;
}
