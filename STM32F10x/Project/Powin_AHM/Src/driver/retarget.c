#pragma import(__use_no_semihosting_swi)

struct __FILE { int handle; /* Add whatever you need here */ };
    FILE __stdout;
    FILE __stdin;
    
int fputc(int ch, FILE *f)
{
    return ITM_SendChar(ch);
}

volatile int32_t ITM_RxBuffer;
int fgetc(FILE *f)
{
  while (ITM_CheckChar() != 1) __NOP();
  return (ITM_ReceiveChar());
}

int ferror(FILE *f)
{
    /* Your implementation of ferror */
    return EOF;
}

void _ttywrch(int c)
{
    fputc(c, 0);
}

int __backspace()
{
    return 0;
}
void _sys_exit(int return_code)
{
label:
    goto label; /* endless loop */
}
