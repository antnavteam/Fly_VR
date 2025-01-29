// pch.h : Il s'agit d'un fichier d'en-tête précompilé.
// Les fichiers listés ci-dessous sont compilés une seule fois, ce qui améliore les performances de génération des futures builds.
// Cela affecte également les performances d'IntelliSense, notamment la complétion du code et de nombreuses fonctionnalités de navigation du code.
// Toutefois, les fichiers listés ici sont TOUS recompilés si l'un d'entre eux est mis à jour entre les builds.
// N'ajoutez pas de fichiers fréquemment mis à jour ici, car cela annule les gains de performance.

#ifndef PCH_H
#define PCH_H

// ajouter les en-têtes à précompiler ici
#include "framework.h"

struct Struct_result
{
    double headingAngle;
    double frame_rate;
    int wings;
    unsigned char* display_image;  // Pointer to the image data
    int display_image_size;        // Size of the image data in bytes
};

typedef void (*CallbackFunction)(Struct_result struct_result);
extern "C" __declspec(dllexport) int AcquireImages(CallbackFunction callback);

#endif //PCH_H
