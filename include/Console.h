#ifndef Console_h
#define Console_h

template <typename T>
void consoleInternal(const char *name, T value);
template <typename T, typename... Args>
void consoleInternal(const char *name, T value, Args... args);
template <typename... Args>
void console(Args... args);

#endif