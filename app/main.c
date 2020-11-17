#include <stdio.h>
int main(){
  int i;
  int sum=0;
  for(i=0; i<=1000; i++) sum+= i;

  printf("%x %d\n", sum, sum);
  return 0;
}
