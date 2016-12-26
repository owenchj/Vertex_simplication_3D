Informatique Graphique 3D et Réalité Virtuelle 

Projet VSA:

make -f Makefile.linux clean <br>
make -f Makefile.linux <br>
./main models/tweety.off<br>

Key 'r' : voir les anchor vertex                   (pointes)<br>
Key 'l' : voir le contour d'objet avec des lignes  (lignes)<br>
Key 'o' : voir l'objet orignal                     (triangles)<br>
Key 't' : voir le résultat de remesh               (triangles)<br>




S'il y a des erreurs, execute plusieurs fois, merci. <br>
S'il vous avez des problème avec execution de programme,<br>
me contactez sur l'email: owenchj@gmail.com <br>


## Result :

### bunny.off, 35947 vertex; 69451 triangles; 100 proxies; 10 iterations; about 5 minute
| Partitionnement | Bords des proxies | Remesh |
| ---- | ---- | ---- |
![bunny0](https://github.com/owenchj/Vertex_simplication_3D/blob/master/doc/img/bunny0.jpg) | ![bunny1](https://github.com/owenchj/Vertex_simplication_3D/blob/master/doc/img/bunny1.jpg) | ![bunny2](https://github.com/owenchj/Vertex_simplication_3D/blob/master/doc/img/bunny2.jpg)

### tweety.off, 7053 vertex; 14102 triangles; 100 proxies; 10 iterations; 18 seconds
| Partitionnement | Bords des proxies | Remesh |
| ---- | ---- | ---- |
![tweety0](https://github.com/owenchj/Vertex_simplication_3D/blob/master/doc/img/tweety0.jpg) | ![tweety1](https://github.com/owenchj/Vertex_simplication_3D/blob/master/doc/img/tweety1.jpg) | ![tweety2](https://github.com/owenchj/Vertex_simplication_3D/blob/master/doc/img/tweety2.jpg)

## Reference
[1] COHEN­STEINER D., ALLIEZ P., DESBRUN M.: Variational shape approximation. ACM
    Transactions on Graphics (Proc. Siggraph) 23, 3 (2004), 905–914. 