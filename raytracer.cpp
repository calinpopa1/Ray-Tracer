#include <cstdio>
#include <cstdlib>
#include <cfloat>
#include <cmath>
#include <algorithm>
#include <string>
#include <fstream>
#include <vector>
#include <iostream>
#include <sstream>
#include <map>
#include <vector>

#include "raytracer.hpp"
#include "image.hpp"


void Raytracer::render(const char *filename, const char *depth_filename,
                       Scene const &scene)
{
    // Alloue les deux images qui seront sauvegardées à la fin du programme.
    Image colorImage(scene.resolution[0], scene.resolution[1]);
    Image depthImage(scene.resolution[0], scene.resolution[1]);
    
    // Crée le zBuffer.
    double *zBuffer = new double[scene.resolution[0] * scene.resolution[1]];
    for(int i = 0; i < scene.resolution[0] * scene.resolution[1]; i++) {
        zBuffer[i] = DBL_MAX;
    }

	// @@@@@@ VOTRE CODE ICI
	// Calculez les paramètres de la caméra pour les rayons. Référez-vous aux slides pour les détails.
	//!!! NOTE UTILE : tan() prend des radians plutot que des degrés. Utilisez deg2rad() pour la conversion.
	//!!! NOTE UTILE : Le plan de vue peut être n'importe où, mais il sera implémenté différement.
	// Vous trouverez des références dans le cours.
	//Matrix m;
	// compute the basis vectors.
	Vector v = scene.camera.up;
	v.normalize();
	Vector w =  scene.camera.center - scene.camera.position ;
	w.normalize();
	Vector u =  w.cross(v);
	u.normalize();

	double t = tan(deg2rad(scene.camera.fovy / 2)) * scene.camera.zNear;
	double b = -t;
	double r = t*scene.camera.aspect;
	double l = -r;

    // Itère sur tous les pixels de l'image.
    for(int y = 0; y < scene.resolution[1]; y++) {
        for(int x = 0; x < scene.resolution[0]; x++) {

            // Génère le rayon approprié pour ce pixel.
			Ray ray;
			if (scene.objects.empty())
			{
				// Pas d'objet dans la scène --> on rend la scène par défaut.
				// Pour celle-ci, le plan de vue est à z = 640 avec une largeur et une hauteur toute deux à 640 pixels.
				ray = Ray(scene.camera.position, (Vector(-320, -320, 640) + Vector(x + 0.5, y + 0.5, 0) - scene.camera.position).normalized());
			}
			else
			{
				// @@@@@@ VOTRE CODE ICI
				// Mettez en place le rayon primaire en utilisant les paramètres de la caméra.
				//!!! NOTE UTILE : tous les rayons dont les coordonnées sont exprimées dans le
				//                 repère monde doivent avoir une direction normalisée.
				//position=eye,center=ref,up=up
				

				Vector origin = scene.camera.position + scene.camera.zNear*w + l*u + b*v;
				Vector Pij = origin + (x + 0.5) * (r - l) / scene.resolution[0] * u + (y + 0.5) * (t - b) / scene.resolution[1] * v;
				

				ray = Ray(scene.camera.position, (Pij - scene.camera.position).normalized());
				
			}

            // Initialise la profondeur de récursivité du rayon.
            int rayDepth = 0;
           
            // Notre lancer de rayons récursif calculera la couleur et la z-profondeur.
            Vector color;

            // Ceci devrait être la profondeur maximum, correspondant à l'arrière plan.
            // NOTE : Ceci suppose que la direction du rayon est de longueur unitaire (normalisée)
			//        et que l'origine du rayon est à la position de la caméra.
            double depth = scene.camera.zFar;

            // Calcule la valeur du pixel en lançant le rayon dans la scène.
            trace(ray, rayDepth, scene, color, depth);

            // Test de profondeur
            if(depth >= scene.camera.zNear && depth <= scene.camera.zFar && 
                depth < zBuffer[x + y*scene.resolution[0]]) {
                zBuffer[x + y*scene.resolution[0]] = depth;

                // Met à jour la couleur de l'image (et sa profondeur)
                colorImage.setPixel(x, y, color);
                depthImage.setPixel(x, y, (depth-scene.camera.zNear) / 
                                        (scene.camera.zFar-scene.camera.zNear));
            }
        }

		// Affiche les informations de l'étape
		if (y % 100 == 0)
		{
			printf("Row %d pixels finished.\n", y);
		}
    }

	// Sauvegarde l'image
    colorImage.writeBMP(filename);
    depthImage.writeBMP(depth_filename);

	printf("Ray tracing finished with images saved.\n");

    delete[] zBuffer;
}


bool Raytracer::trace(Ray const &ray, 
                 int &rayDepth,
                 Scene const &scene,
                 Vector &outColor, double &depth)
{
    // Incrémente la profondeur du rayon.
    rayDepth++;

    // - itérer sur tous les objets en appelant   Object::intersect.
    // - ne pas accepter les intersections plus lointaines que la profondeur donnée.
    // - appeler Raytracer::shade avec l'intersection la plus proche.
    // - renvoyer true ssi le rayon intersecte un objet.
	if (scene.objects.empty())
	{
		// Pas d'objet dans la scène --> on rend la scène par défaut :
		// Par défaut, un cube est centré en (0, 0, 1280 + 160) avec une longueur de côté de 320, juste en face de la caméra.
		// Test d'intersection :
		double x = 1280 / ray.direction[2] * ray.direction[0] + ray.origin[0];
		double y = 1280 / ray.direction[2] * ray.direction[1] + ray.origin[1];
		if ((x <= 160) && (x >= -160) && (y <= 160) && (y >= -160))
		{
			// S'il y a intersection :
			Material m; m.emission = Vector(16.0, 0, 0); m.reflect = 0; // seulement pour le matériau par défaut ; vous devrez utiliser le matériau de l'objet intersecté
			Intersection intersection;	// seulement par défaut ; vous devrez passer l'intersection trouvée par l'appel à Object::intersect()
			outColor = shade(ray, rayDepth, intersection, m, scene);
			depth = 1280;	// la profondeur devrait être mise à jour dans la méthode Object::intersect()
		}
	}
	else
	{
		// @@@@@@ VOTRE CODE ICI
		// Notez que pour Object::intersect(), le paramètre hit correspond à celui courant.
		// Votre intersect() devrait être implémenté pour exclure toute intersection plus lointaine que hit.depth
		Intersection near_int;
		Material mater;
		
		
		for (auto objIter = scene.objects.begin(); objIter != scene.objects.end(); objIter++)
		{
			Intersection intersection;
			if ((*objIter)->intersect(ray, intersection)) {
				if (intersection.depth <= depth) {
					depth = intersection.depth;
					near_int = intersection;
					mater = (*objIter)->material;
				}
			}
		}
		if (depth < scene.camera.zFar) {
			outColor = shade(ray, rayDepth, near_int, mater, scene);
		}
		else {
			outColor = Vector(0, 0, 0);
		}
		
	}

    // Décrémente la profondeur du rayon.
    rayDepth--;

    return false; 
}


Vector Raytracer::shade(Ray const &ray,
                 int &rayDepth,
                 Intersection const &intersection,
                 Material const &material,
                 Scene const &scene)
{
    // - itérer sur toutes les sources de lumières, calculant les contributions ambiant/diffuse/speculaire
    // - utiliser les rayons d'ombre pour déterminer les ombres
    // - intégrer la contribution de chaque lumière
    // - inclure l'émission du matériau de la surface, s'il y a lieu
    // - appeler Raytracer::trace pour les couleurs de reflection/refraction
    // Ne pas réfléchir/réfracter si la profondeur de récursion maximum du rayon a été atteinte !
	//!!! NOTE UTILE : facteur d'atténuation = 1.0 / (a0 + a1 * d + a2 * d * d)..., la lumière ambiante ne s'atténue pas, ni n'est affectée par les ombres
	//!!! NOTE UTILE : n'acceptez pas les intersection des rayons d'ombre qui sont plus loin que la position de la lumière
	//!!! NOTE UTILE : pour chaque type de rayon, i.e. rayon d'ombre, rayon reflechi, et rayon primaire, les profondeurs maximales sont différentes
	Vector diffuse(0);
	Vector ambient(0);
	Vector specular(0);
	for (auto lightIter = scene.lights.begin(); lightIter != scene.lights.end(); lightIter++)
	{
		// @@@@@@ VOTRE CODE ICI
		// Calculez l'illumination locale ici, souvenez-vous d'ajouter les lumières ensemble.
		// Testez également les ombres ici, si un point est dans l'ombre aucune couleur (sauf le terme ambient) ne devrait être émise.
		//diffuse += Vector(1,1,1)
		//Vector relativeVertexPosition = ray * lightIter->position;
		//Vector vertPos = relativeVertexPosition / relativeVertexPosition[3];

		Vector V = -ray.direction.normalized();
		Vector L = (lightIter->position - intersection.position).normalized();
		double Ldist = L.length();
		Vector N = intersection.normal.normalized();
		Vector H = (((L + V) / 2)).normalized();
		double attenuation = 1.0 / (lightIter->attenuation[0] + lightIter->attenuation[1] * Ldist + lightIter->attenuation[2] * Ldist * Ldist);

		double lambertian = std::max(N.dot(L), 0.0);
		double spec = 0.0;
		

		if (lambertian > 0.0) {
			double specAngle = std::max(H.dot(N), 0.0);
			spec = std::pow(specAngle,material.shininess );
		}
		
		 Vector ambient_light = material.ambient * lightIter->ambient;
		 Vector diffuse_light= material.diffuse * lightIter->diffuse * attenuation * lambertian;
		 Vector specular_light = material.specular * lightIter->specular * attenuation * spec;
		
		Ray shadowRay = Ray(intersection.position + L * 1e-6, L);
		Intersection hit = intersection;
		bool inShadow = false;

		for (auto objIter = scene.objects.begin(); objIter != scene.objects.end(); objIter++) {
			if ((*objIter)->intersect(shadowRay, hit)) {
				inShadow = true;
			}
		}

		if (inShadow) {
			ambient += ambient_light;
			diffuse += 0;
			specular += 0;
		}
		else {
			ambient += ambient_light;
			diffuse += diffuse_light;
			specular += specular_light;
		}


		




		//Vector normalInterp = intersection.normal;

		//Vector N = normalInterp.normalized();
		//Vector L = -(ray * lightIter->position.normalized();

		//double lambertian = std::max(N.dot(L), 0.0);
		//double specular = 0.0;

		//if (lambertian > 0.0) {
		//	Vector R = (-L).reflect(N);
		//	Vector V = (-vertPos).normalized();
		//	double specAngle = std::max(R.dot(V), 0.0);
		//	specular = pow(specAngle, material.shininess);
		//}

		//Vector color = Vector(lightIter->ambient + lambertian * lightIter->specular * diffuse + lightIter->specular, 1.0);
		//lightIter->position

		//Idiffuse=

		//Vector Émis = material.emission + lightIter->ambient + lightIter->attenuation + lightIter->diffuse;

	}

	Vector reflectedLight(0);
	if ((!(ABS_FLOAT(material.reflect) < 1e-6)) && (rayDepth < MAX_RAY_RECURSION))
	{
		// @@@@@@ VOTRE CODE ICI
		// Calculez la couleur réfléchie en utilisant trace() de manière récursive.
		Vector lumiere = -ray.direction.normalized();
		Vector nor = intersection.normal.normalized();
		Vector reflection = (-lumiere + 2 * (lumiere.dot(nor)) * nor).normalized();

		Ray rayreflected(intersection.position + reflection * 1e-6, reflection);
		double depth = DBL_MAX;
		trace(rayreflected, rayDepth, scene, reflectedLight, depth);
	}

	return material.emission + ambient + diffuse + specular + material.reflect * reflectedLight;
}