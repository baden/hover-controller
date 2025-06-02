volatile int a = 7, b = 3, c, d;

void test(void) {
    c = a * b;  // має бути MUL
    d = a / b;  // має бути програмне ділення
}
