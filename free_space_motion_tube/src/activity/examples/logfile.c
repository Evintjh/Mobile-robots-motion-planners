// Free space
#include <free_space_motion_tube/free_space.h>
#include <free_space_motion_tube/basic.h>

#include <stdio.h>
#include <stdlib.h>

int check_if_end_of_file(FILE *file);
void print_array(int n_values, double *array);

int main(int argc, char **argv){
   if (argc != 2){
       printf("Wrong number of arguments");
       exit(1);
   }
   
    char *filename = argv[1];
    FILE *myFile;
    myFile = fopen(filename, "r");

   if (myFile == NULL){
       printf("Error! opening file");
       // Program exits if the file pointer returns NULL.
       exit(1);
   }

    int nb_elements, end;
    double *ranges, *angles;
    while(1) {
        // Read ranges
        fscanf(myFile, "%d", &nb_elements);
        ranges = (double *) malloc(sizeof(double) * nb_elements);
        for(int i=0; i<nb_elements; i++)
            fscanf(myFile, "%lf", &(ranges[i]));

        // Read angles
        fscanf(myFile, "%d", &nb_elements);
        angles = (double *) malloc(sizeof(double) * nb_elements);
        for(int i=0; i<nb_elements; i++)
            fscanf(myFile, "%lf", &(angles[i]));

        // Do some cool stuff here..

        free(ranges);
        free(angles);

        if(check_if_end_of_file(myFile)){
            break;
        }
    }
    return 0;
}

int check_if_end_of_file(FILE *file) {
    char c = fgetc(file);
    return (c == EOF);
}

void print_array(int n_values, double *array) {
    int i;
    for (i = 0; i < n_values; i++) {
        printf("%lf\t", array[i]);
    }
    printf("\n");
}