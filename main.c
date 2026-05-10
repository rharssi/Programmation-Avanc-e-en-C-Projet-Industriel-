/*
 * Détection de collision pour un essaim de 10 000 UAVs
 * Pr. Tarik HOUICHIME – École des Sciences de l'Information
 *
 * Règles du projet :
 *   - pas de [] pour accéder aux tableaux, que de l'arithmétique de pointeurs
 *   - un seul malloc par buffer
 *   - on vise O(n log n) au total
 *
 * Approche :
 *   1. Tri fusion sur X
 *   2. Sweep line pour trouver la paire la plus proche
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <time.h>

/* Un drone dans l'espace 3D */
struct Drone {
    int   id;
    float x, y, z;  /* coordonnées en mètres */
};

/* Distance euclidienne entre deux drones */
static float calculerDistance(const struct Drone *a, const struct Drone *b)
{
    float dx = a->x - b->x;
    float dy = a->y - b->y;
    float dz = a->z - b->z;
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

/*
 * Fusion classique du merge sort.
 * base[gauche..milieu] et base[milieu+1..droite] sont déjà triés par x.
 * On fusionne dans tmp puis on recopie.
 *
 * base est un tableau de pointeurs vers les Drones (pas les structs directement),
 * donc on ne déplace que des adresses de 8 octets au lieu de structs entières.
 */
static void fusionner(struct Drone **base,
                      int gauche, int milieu, int droite,
                      struct Drone *tmp)
{
    int i = gauche;       /* curseur côté gauche */
    int j = milieu + 1;   /* curseur côté droit  */
    int k = gauche;       /* curseur dans tmp    */

    /* on intercale les deux moitiés dans tmp */
    while (i <= milieu && j <= droite) {
        if ((*(base + i))->x <= (*(base + j))->x) {
            *(tmp + k) = *(*(base + i));
            i++;
        } else {
            *(tmp + k) = *(*(base + j));
            j++;
        }
        k++;
    }

    /* reste du côté gauche */
    while (i <= milieu) {
        *(tmp + k) = *(*(base + i));
        i++; k++;
    }

    /* reste du côté droit */
    while (j <= droite) {
        *(tmp + k) = *(*(base + j));
        j++; k++;
    }

    /* on remet le résultat trié dans l'entrepôt d'origine */
    for (int l = gauche; l <= droite; l++) {
        *(*(base + l)) = *(tmp + l);
    }
}

/* Tri fusion récursif sur le tableau de pointeurs */
static void triParFusion(struct Drone **ptrs,
                          int gauche, int droite,
                          struct Drone *tmp)
{
    if (gauche >= droite) return;

    int milieu = gauche + (droite - gauche) / 2;  /* évite l'overflow */

    triParFusion(ptrs, gauche,    milieu, tmp);
    triParFusion(ptrs, milieu+1,  droite, tmp);
    fusionner   (ptrs, gauche,    milieu, droite, tmp);
}

/*
 * Cherche la paire de drones la plus proche après tri par x.
 *
 * Principe : pour chaque drone A, on regarde ses voisins de droite B.
 * Dès que B.x - A.x >= d_min courante, on s'arrête — inutile d'aller
 * plus loin, la distance 3D ne peut qu'être plus grande.
 * C'est cette coupure qui nous évite de faire O(n²) comparaisons.
 *
 * Retourne la distance minimale et remplit id1/id2.
 */
static float balayageFenetreActive(struct Drone *essaim,
                                    int n,
                                    int *id1, int *id2)
{
    float d_min = FLT_MAX;
    *id1 = -1;
    *id2 = -1;

    struct Drone *ptr_A, *ptr_B;

    for (int i = 0; i < n - 1; i++) {
        ptr_A = essaim + i;

        for (int j = i + 1; j < n; j++) {
            ptr_B = essaim + j;

            /* coupure sur l'axe X — on arrête dès que c'est inutile */
            float delta_x = ptr_B->x - ptr_A->x;
            if (delta_x >= d_min) break;

            float d = calculerDistance(ptr_A, ptr_B);
            if (d < d_min) {
                d_min = d;
                *id1  = ptr_A->id;
                *id2  = ptr_B->id;
            }
        }
    }

    return d_min;
}

/* Remplit l'essaim avec des positions aléatoires dans un cube 10km x 10km x 1km */
static void genererEssaim(struct Drone *essaim, int n)
{
    struct Drone *ptr = essaim;
    for (int i = 0; i < n; i++) {
        ptr->id = i;
        ptr->x  = (float)(rand() % 10000);
        ptr->y  = (float)(rand() % 10000);
        ptr->z  = (float)(rand() % 1000);
        ptr++;
    }
}

int main(void)
{
    const int N = 10000;

    printf("=== SYSTÈME DE COLLISION UAV – ESSAIM DE %d DRONES ===\n\n", N);

    /* -- allocation de l'entrepôt principal -- */
    struct Drone *essaim = (struct Drone *)malloc((size_t)N * sizeof(struct Drone));
    if (essaim == NULL) {
        fprintf(stderr, "[ERREUR] malloc échoué pour l'essaim\n");
        return EXIT_FAILURE;
    }
    printf("[OK] Mémoire allouée : %zu octets\n", (size_t)N * sizeof(struct Drone));

    srand((unsigned int)time(NULL));
    genererEssaim(essaim, N);
    printf("[OK] %d drones générés\n", N);

    /* tableau de pointeurs pour le tri — on déplace des adresses, pas des structs */
    struct Drone **ptrs = (struct Drone **)malloc((size_t)N * sizeof(struct Drone *));
    if (ptrs == NULL) {
        fprintf(stderr, "[ERREUR] malloc échoué pour ptrs\n");
        free(essaim);
        return EXIT_FAILURE;
    }

    for (int i = 0; i < N; i++)
        *(ptrs + i) = essaim + i;

    /* buffer temporaire pour la phase de fusion */
    struct Drone *tmp = (struct Drone *)malloc((size_t)N * sizeof(struct Drone));
    if (tmp == NULL) {
        fprintf(stderr, "[ERREUR] malloc échoué pour tmp\n");
        free(ptrs);
        free(essaim);
        return EXIT_FAILURE;
    }

    /* -- tri par x -- */
    printf("[...] Tri en cours...\n");
    clock_t debut = clock();
    triParFusion(ptrs, 0, N - 1, tmp);
    clock_t fin_tri = clock();
    double temps_tri = (double)(fin_tri - debut) / CLOCKS_PER_SEC * 1000.0;
    printf("[OK] Tri terminé en %.3f ms\n", temps_tri);

    /* on remet les drones dans l'ordre trié dans essaim pour un accès contigu */
    for (int i = 0; i < N; i++) *(tmp + i)    = *(*(ptrs + i));
    for (int i = 0; i < N; i++) *(essaim + i) = *(tmp + i);

    /* -- sweep line -- */
    printf("[...] Recherche de la paire la plus proche...\n");
    clock_t debut_sweep = clock();
    int drone_id1 = -1, drone_id2 = -1;
    float d_min = balayageFenetreActive(essaim, N, &drone_id1, &drone_id2);
    clock_t fin_sweep = clock();
    double temps_sweep = (double)(fin_sweep - debut_sweep) / CLOCKS_PER_SEC * 1000.0;
    printf("[OK] Balayage terminé en %.3f ms\n", temps_sweep);

    /* -- affichage du résultat -- */
    printf("\n╔══════════════════════════════════════════════════╗\n");
    printf("║        RÉSULTAT : PAIRE CRITIQUE DÉTECTÉE        ║\n");
    printf("╠══════════════════════════════════════════════════╣\n");

    if (drone_id1 >= 0 && drone_id2 >= 0) {
        struct Drone *d1 = NULL, *d2 = NULL;
        for (int i = 0; i < N; i++) {
            if ((essaim + i)->id == drone_id1) d1 = essaim + i;
            if ((essaim + i)->id == drone_id2) d2 = essaim + i;
        }
        printf("║  Drone A : ID=%-5d  (%.1f, %.1f, %.1f) m\n",
               drone_id1, d1->x, d1->y, d1->z);
        printf("║  Drone B : ID=%-5d  (%.1f, %.1f, %.1f) m\n",
               drone_id2, d2->x, d2->y, d2->z);
        printf("║  Distance minimale : %.4f m\n", d_min);
        printf("╠══════════════════════════════════════════════════╣\n");
        printf("║  ⚠  MANŒUVRE D'ÉVITEMENT DÉCLENCHÉE             ║\n");
    } else {
        printf("║  Aucune paire critique détectée.                  ║\n");
    }

    printf("╚══════════════════════════════════════════════════╝\n");

    printf("\n── Bilan des Performances ──────────────────────────\n");
    printf("  Temps total     : %.3f ms\n", temps_tri + temps_sweep);
    printf("  Temps tri       : %.3f ms\n", temps_tri);
    printf("  Temps balayage  : %.3f ms\n", temps_sweep);

    free(tmp);
    free(ptrs);
    free(essaim);
    printf("\n[OK] Mémoire libérée.\n");

    return EXIT_SUCCESS;
}

