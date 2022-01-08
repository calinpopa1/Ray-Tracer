#include "object.hpp"

#include <cmath>
#include <cfloat>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <iostream>


bool Object::intersect(Ray ray, Intersection &hit) const 
{
    // Assure une valeur correcte pour la coordonnée W de l'origine et de la direction
	// Vous pouvez commentez ces lignes si vous faites très attention à la façon de construire vos rayons.
    ray.origin[3] = 1;
    ray.direction[3] = 0;

    Ray local_ray(i_transform * ray.origin, i_transform * ray.direction);
	//!!! NOTE UTILE : pour calculer la profondeur dans localIntersect(), si l'intersection se passe à
	// ray.origin + ray.direction * t, alors t est la profondeur
	//!!! NOTE UTILE : ici, la direction peut êytre mise à l'échelle, alors vous devez la renormaliser
	// dans localIntersect(), ou vous aurez une profondeur dans le système de coordonnées local, qui
	// ne pourra pas être comparée aux intersection avec les autres objets.
    if (localIntersect(local_ray, hit)) 
	{
        // Assure la valeur correcte de W.
        hit.position[3] = 1;
        hit.normal[3] = 0;
        
		// Transforme les coordonnées de l'intersection dans le repère global.
        hit.position = transform * hit.position;
        hit.normal = (n_transform * hit.normal).normalized();
        
		return true;
    }

    return false;
}


bool Sphere::localIntersect(Ray const &ray, Intersection &hit) const 
{
    // @@@@@@ VOTRE CODE ICI
	// Vous pourriez aussi utiliser des relations géométriques pures plutôt que les
	// outils analytiques présentés dans les slides.
	// Ici, dans le système de coordonées local, la sphère est centrée en (0, 0, 0)
	//
	// NOTE : hit.depth est la profondeur de l'intersection actuellement la plus proche,
	// donc n'acceptez pas les intersections qui occurent plus loin que cette valeur.
	double a = ray.direction[0] * ray.direction[0] + ray.direction[1] * ray.direction[1] + ray.direction[2] * ray.direction[2];
	double b = 2 * (ray.origin[0] * ray.direction[0] + ray.origin[1] * ray.direction[1] + ray.origin[2] * ray.direction[2]);
	double c = ray.origin[0] * ray.origin[0] + ray.origin[1] * ray.origin[1] + ray.origin[2] * ray.origin[2] - (this->radius) * (this->radius);
	
	double discriminant = b * b - 4 * a * c;


	if (discriminant > 0.0) {
		double t1 = (-b + sqrt(discriminant)) / (2 * a);
		double t2 = (-b - sqrt(discriminant)) / (2 * a);
		if (0.0 < t1 ) {
			if (0.0 < t2) {
				Vector p = ray.origin + std::min(t1,t2) * ray.direction;
				hit.depth = std::min(t1, t2);
				hit.normal = (p - Vector(0.0, 0.0, 0.0)).normalized();
				hit.position = ray.origin + std::min(t1, t2) * ray.direction;
				return true;
			}
			
		}
		if (0.0 < t1 && 0.0 > t2) {
			Vector p = ray.origin + t1 * ray.direction;
			hit.depth = t1;
			hit.normal = (p-Vector(0.0, 0.0, 0.0) ).normalized();
			hit.position = ray.origin + t1 * ray.direction;
			return true;
		}
		if (0.0 < t2 && 0.0 > t1) {
			Vector p = ray.origin + t2 * ray.direction;
			hit.depth = t2;
			hit.normal = p.normalized();
			hit.position = ray.origin + t2 * ray.direction;
			return true;
		}

	
	}
	return false;

	
	
}


bool Plane::localIntersect(Ray const &ray, Intersection &hit) const
{
	// @@@@@@ VOTRE CODE ICI
	// N'acceptez pas les intersections tant que le rayon est à l'intérieur du plan.
	// ici, dans le système de coordonées local, le plan est à z = 0.
	//
	// NOTE : hit.depth est la profondeur de l'intersection actuellement la plus proche,
	// donc n'acceptez pas les intersections qui occurent plus loin que cette valeur.
	Vector planenormal = Vector(0.0, 0.0, 1.0);
	Vector planecenter = Vector(0.0, 0.0, 0.0);
	
	double denom = planenormal.dot(ray.direction);
	if (denom != 0.0) {
		double t = (planecenter - ray.origin).dot(planenormal) / denom;
		if (t >= 0.0 && t<hit.depth) {
			
			hit.depth = t;
			hit.normal = planenormal;
			hit.position = ray.origin + t * ray.direction;
			return true;
		}
	}
    return false;
}


bool Conic::localIntersect(Ray const &ray, Intersection &hit) const {
    // @@@@@@ VOTRE CODE ICI (licence créative)
	double a = ray.direction[0] * ray.direction[0] + ray.direction[1] * ray.direction[1] + ray.direction[2] * ray.direction[2];
	double b = 2 * (ray.origin[0] * ray.direction[0] + ray.origin[1] * ray.direction[1] + ray.origin[2] * ray.direction[2]);
	double c = ray.origin[0] * ray.origin[0] + ray.origin[1] * ray.origin[1] + ray.origin[2] * ray.origin[2] - (this->radius1) * (this->radius1);

	double discriminant = b * b - 4 * a * c;

	double t1 = (-b + discriminant * (1 / 2)) / (2 * a);
	double t2 = (-b - discriminant * (1 / 2)) / (2 * a);



	

	if (discriminant > 0.0) {
		
		if (0.0 < t1) {
			if (0.0 < t2) {
				Vector p = ray.origin + std::min(t1, t2) * ray.direction;
				hit.depth = std::min(t1, t2);
				hit.normal = (p - Vector(0.0, 0.0, 0.0)).normalized();
				hit.position = ray.origin + std::min(t1, t2) * ray.direction;
				return true;
			}

		}
		if (0.0 < t1 && 0.0 > t2) {
			Vector p = ray.origin + t1 * ray.direction;
			hit.depth = t1;
			hit.normal = (p - Vector(0.0, 0.0, 0.0)).normalized();
			hit.position = ray.origin + t1 * ray.direction;
			return true;
		}
		if (0.0 < t2 && 0.0 > t1) {
			Vector p = ray.origin + t2 * ray.direction;
			hit.depth = t2;
			hit.normal = p.normalized();
			hit.position = ray.origin + t2 * ray.direction;
			return true;
		}


	}
	return false;
}


// Intersections !
bool Mesh::localIntersect(Ray const &ray, Intersection &hit) const
{
	// Test de la boite englobante
	double tNear = -DBL_MAX, tFar = DBL_MAX;
	for (int i = 0; i < 3; i++) {
		if (ray.direction[i] == 0.0) {
			if (ray.origin[i] < bboxMin[i] || ray.origin[i] > bboxMax[i]) {
				// Rayon parallèle à un plan de la boite englobante et en dehors de la boite
				return false;
			}
			// Rayon parallèle à un plan de la boite et dans la boite: on continue
		}
		else {
			double t1 = (bboxMin[i] - ray.origin[i]) / ray.direction[i];
			double t2 = (bboxMax[i] - ray.origin[i]) / ray.direction[i];
			if (t1 > t2) std::swap(t1, t2); // Assure t1 <= t2

			if (t1 > tNear) tNear = t1; // On veut le plus lointain tNear.
			if (t2 < tFar) tFar = t2; // On veut le plus proche tFar.

			if (tNear > tFar) return false; // Le rayon rate la boite englobante.
			if (tFar < 0) return false; // La boite englobante est derrière le rayon.
		}
	}
	// Si on arrive jusqu'ici, c'est que le rayon a intersecté la boite englobante.

	// Le rayon interesecte la boite englobante, donc on teste chaque triangle.
	bool isHit = false;
	for (size_t tri_i = 0; tri_i < triangles.size(); tri_i++) {
		Triangle const &tri = triangles[tri_i];

		if (intersectTriangle(ray, tri, hit)) {
			isHit = true;
		}
	}
	return isHit;
}

double Mesh::implicitLineEquation(double p_x, double p_y,
	double e1_x, double e1_y,
	double e2_x, double e2_y) const
{
	return (e2_y - e1_y)*(p_x - e1_x) - (e2_x - e1_x)*(p_y - e1_y);
}

bool Mesh::intersectTriangle(Ray const &ray,
	Triangle const &tri,
	Intersection &hit) const
{
	// Extrait chaque position de sommet des données du maillage.
	Vector const &p0 = positions[tri[0].pi];
	Vector const &p1 = positions[tri[1].pi];
	Vector const &p2 = positions[tri[2].pi];

	// @@@@@@ VOTRE CODE ICI
	// Décidez si le rayon intersecte le triangle (p0,p1,p2).
	// Si c'est le cas, remplissez la structure hit avec les informations
	// de l'intersection et renvoyez true.
	// Vous pourriez trouver utile d'utiliser la routine implicitLineEquation()
	// pour calculer le résultat de l'équation de ligne implicite en 2D.
	//
	// NOTE : hit.depth est la profondeur de l'intersection actuellement la plus proche,
	// donc n'acceptez pas les intersections qui occurent plus loin que cette valeur.
	//!!! NOTE UTILE : pour le point d'intersection, sa normale doit satisfaire hit.normal.dot(ray.direction) < 0

	Vector vec01 = p1 - p0;
	Vector vec12 = p2 - p1;
	Vector vec20 = p0 - p2;

	Vector normal = vec01.cross(vec12);

	double d = -(normal.dot(p0)) ;

	double t = -((normal.dot(ray.origin) + d) / (normal.dot(ray.direction)));
	

	if (t > hit.depth || t < 0.0) {
		return false;
	}
	Vector p = ray.origin + t * ray.direction;
	Vector vec0t = p - p0;
	Vector vec1t = p - p1;
	Vector vec2t = p - p2;

	Vector alpha = vec01.cross(vec0t);
	Vector beta = vec12.cross(vec1t);
	Vector gamma = vec20.cross(vec2t);

	if ((alpha.dot(beta) >= 0.0) && (alpha.dot(gamma) >= 0.0 )&& (beta.dot(gamma) >= 0.0) 
		|| (alpha.dot(beta) < 0.0) && (alpha.dot(gamma) < 0.0) && (beta.dot(gamma) < 0.0)) {
		hit.depth = t;
		hit.normal = normal;
		hit.position = ray.origin + t * ray.direction;
		
		return true;
	}
	return false;

	
}
