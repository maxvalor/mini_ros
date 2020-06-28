#include <memory>
#include <iostream>

class base{
 public:
    base(){std::cout << "base" << std::endl;}
    ~base(){std::cout << "~base" << std::endl;}
    void print(){std::cout << "base::print" << std::endl;}
};

class derived:public base{
 public:
    derived(){std::cout << "derived" << std::endl;}
    ~derived(){std::cout << "~derived" << std::endl;}
    void print(){std::cout << "derived::print" << std::endl;}
};

int main()
{
    std::shared_ptr<base> b_ptr = std::make_shared<derived>();
    b_ptr->print();
    auto d_ptr = std::static_pointer_cast<derived>(b_ptr);
    d_ptr->print();
    return 0;
}
