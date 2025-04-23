
#include <stdio.h>

#define CHECK(x) if (!(x)) printf("CHECK failed %d %s\n", __LINE__, #x)


int main(void)
{
  CHECK(1 == 0);
  CHECK(17);
  CHECK(1+1-2);

  return 0;
}

