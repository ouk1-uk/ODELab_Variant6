#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

void throwInteger() { throw 42; }
void throwString() { throw std::string("An error represented by std::string"); }
void throwStandard() { throw std::invalid_argument("A standard exception"); }

int main() {
    std::cout << "--- Throwing int ---\n";
    try { throwInteger(); }
    catch (const std::exception& ex) { std::cout << "std::exception: " << ex.what() << '\n'; }
    catch (...) { std::cout << "catch(...) caught int (std::exception did not)\n"; }

    std::cout << "--- Throwing std::string ---\n";
    try { throwString(); }
    catch (const std::exception& ex) { std::cout << "std::exception: " << ex.what() << '\n'; }
    catch (...) { std::cout << "catch(...) caught string (std::exception did not)\n"; }

    std::cout << "--- Throwing std::invalid_argument ---\n";
    try { throwStandard(); }
    catch (const std::invalid_argument& ex) { std::cout << "specific catch: " << ex.what() << '\n'; }
    catch (const std::exception& ex) { std::cout << "generic standard catch: " << ex.what() << '\n'; }
    catch (...) { std::cout << "unknown exception\n"; }
    return 0;
}
