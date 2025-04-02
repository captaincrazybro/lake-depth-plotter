
void generateGrid(float X1, float Y1, float X2, float Y2, int numberPoints, float coordinates[][2]) {
    float xStep = (X2 - X1) / (numberPoints - 1);
    float yStep = (Y2 - Y1) / (numberPoints - 1);

    int index = 0;
    for (int i = 0; i < numberPoints; i++) {
        for (int j = 0; j < numberPoints; j++) {
            coordinates[index][0] = X1 + i * xStep;
            coordinates[index][1] = Y1 + j * yStep;
            index++;
        }
    }
}

void convertCords(char cordInput[], float cords[]){
  char buffer[strlen(cordInput) + 1];
  strcpy(buffer, cordInput);

  char *token = strtok(buffer, ",");
  int index = 0;

  while (token != NULL && index < 4) {
    cords[index] = atof(token);
    token = strtok(NULL, ",");
    index++;
  }
}