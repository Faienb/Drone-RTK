# Projet Drone RTK ToDo liste


Deadlines abstract article :
- 02.01.2021 : Peer Review Paper FIG
- 31.01.2022 : NON Peer review paper FIG

Mesures de contrôle à faire / documentation rapport :
- [x] Valider l'orientation selon la distance en prenant les coordonnées pilier ground truth et que l'orientation qui sort de l'ICP et cheker selon les distances (voir le test_calcul si ca repond a la question)
- [ ] Catégoriser selon les distances et voir si on a des différences significatives (voir si on a un biais)
- [ ] Documenter les précisions et qu'est-ce que quoi ! (les résultats qu'on sort sur la page html -> fitness(nombre d'inlier), inlier_rmse)
- [ ] Pas convaincu que l'ICP est un estimateur non biaisé --> à prouver donc :
- [ ] Argument : mesure à l'heure mais envoi time stampé (je me comprend)

Code contrôles :
- [x] Pourquoi est-ce que le serveur SwiPos veut pas être attaqué à 5Hz (Voir déjà à 2Hz) : possible à 10Hz
- [x] Ouverture et lecture du json est-ce que pourrait être problématique (C'est réglé il ne peut pas y avoir de problèmes)
- [ ] Processus wiki complet --> du hardware configuration à l'implémentation du code
- [ ] Checker le retour tachéo pour avoir une solution sur le temps buffer réel ou possibilité de mesurer précisément le temps de la mesure
- [ ] Estimer le délai moyen de retour (voir la partie ICP avec altitude, c'est fait)
- [ ] Tester perte de gps, buffer serial tacheo (normalement c'est bon car c'est nous qui demandons une mesure, différent du buffer GPS qui mesure de toute façon et nous renvoie)
- [x] Tester GNSS sur un pilier avec un GNSS Leica (Fait, a permis de résoudre notre problème d'altitude)

Finalisation :
- [ ] Nettoyer le code
- [ ] Commenter le code
- [ ] Créer un document PDF de mise en station avec des codes couleurs et toutes les infos importantes, oui mais le mettre ou ?
