#include <stdio.h>


int main(){

    char *fruits[] = {
        "apple",
        "pair",
        "strawberry",
        "grape"
    };

    for (int i = 0; i < 4; i++){
        printf("fruits %d: %s\n", i+1, fruits[i]);
    }
    return 0;
}