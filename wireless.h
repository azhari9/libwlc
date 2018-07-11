#ifdef __cplusplus
extern "C" {
#endif
typedef int (*FuncPtr)(unsigned char *);
extern int subscribeCB (FuncPtr pFunc);
extern int read_UART (void);

#ifdef __cplusplus
}
#endif