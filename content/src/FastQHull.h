/**
 * Author:    Andrea Casalino
 * Created:   03.12.2019
*
* report any bug to andrecasa91@gmail.com.
 **/

#pragma once
#ifndef __FAST_QUICK_HULL_H__
#define __FAST_QUICK_HULL_H__

#include <list>
#include "Hull.h"

#define  GEOMETRIC_TOLLERANCE  float(HULL_GEOMETRIC_TOLLERANCE)

 /* in case OpenMP (https://en.wikipedia.org/wiki/OpenMP) is enabled, Fast_QHull is automatically built in the parallel computaion mode, using a
 number of threads equal to the number of cores available. See also Fast_QHull.pdf
 */
#if defined(_OPENMP)
#include <omp.h>
#endif


 /** \brief The solver to use for computing the convex hull of a point cloud.
 * \details V can be any type representing the 3D Cartesian coordinates of a point in the space.
 typename V must have at least:

 -> A constructor accepting the 3 floats representing the coordinates	V::V(const float& X, const float& Y, const float& Z);

 -> Three getters returning each coordinate, defined in this way (with the obvious meaning of notation):
		 -> const float& V::x() const;
		 -> const float& V::y() const;
		 -> const float& V::z() const;
 */
template<typename V>
class Fast_QHull  {
public:
	Fast_QHull() { this->Vertices = NULL; this->politope = NULL; };
	~Fast_QHull() { if (this->politope != NULL) delete this->politope; };

	/** \brief Use this for computing a new convex hull.
	* \details The passed cloud is analyzed and the incidences (see Fast_QHull.pdf) of the convex hull are internally stored.
	For accessing the incidences use Fast_QHull::Get_Incidences, while the normals of the facet can be obtained by invoking
	Fast_QHull::Get_Normals (the same order of facets is assumed for the incidences and the normals).
	* @param[in] new_set_of_vertices The list of vertices characterizing the point cloud to consider for the convex hull computation (Is the cloud C in the Fast_QHull.pdf)
	* @param[in] max_iterations The maximum number of iterations to consider for updating the convex hull (see Fast_QHull.pdf). Infinity is assumed as default,
	passing 0: the algorithm stops only after the convex hull is completely obtained
	*/
	void Compute_new_Convex_Hull(const std::list<V>* new_set_of_vertices, const size_t& max_iterations = 0);

	/** \brief Returns the incidences (see the documentation) of the convex hull triangulation.
	* \details The incidences are meant here as the positions of the vertex in the cloud passed in Fast_QHull::Compute_new_Convex_Hull. See also Fast_QHull.pdf
	The incidences are computed and internally stored after callling Fast_QHull::Compute_new_Convex_Hull. Therefore,
	when this function is invoked, the incidences of the convex hull of the last passed cloud are returned. In case the convex hull computation
	was not successfull, an empty list is returned.
	* @param[out] indexes The list of incidences characterizing the convex hull
	*/
	void Get_Incidences(std::list<std::list<size_t>>* indexes) const;

	/** \brief Similar to Fast_QHull::Get_Incidences(std::list<std::list<size_t>>* indexes).
	* \details Here the incidences are stored in a vector of positions, which is allocated internally with a malloc operation and then returned. Remember to
	deallocate the buffer of positions after having use it. In case the cinvex hull computation was not successfull, a NULL result is returned
	* @param[out] return The list of incidences characterizing the convex hull
	* @param[out] numb_of_facets the number of facets
	*/
	size_t* Get_Incidences(size_t* numb_of_facets) const;

	/** \brief Returns the incidences of the convex hull triangulation.
	* \details The incidences are computed and internally stored after callling Fast_QHull::Compute_new_Convex_Hull. Therefore,
	when this function is invoked, the incidences of the convex hull of the last passed cloud are returned.  In case the convex hull computation
	was not successfull, an empty list is returned
	* @param[out] normals The list of incidences characterizing the convex hull
	*/
	void Get_Normals(std::list<V>* normals) const;

private:
	struct Facet {
		bool					bVisible; //only for update Hull

		const V*				A; //pointer of the vertex A
		const V*				B; //pointer of the vertex B
		const V*				C; //pointer of the vertex C

		float						N[3]; //outer normal 

		Facet*				Neighbour[3]; // AB, BC, CA

		size_t					id_A; //pos in the list of vertices of point A
		size_t					id_B; //pos in the list of vertices of point B
		size_t					id_C; //pos in the list of vertices of point C

		const V*				Furthest_vertex; //if different form NULL this facet can be considered for further expansion of the hull
		size_t					Furthest_Pos;
		float						Dist_Furthest_vertex;
	};
	class Politope : public Hull<Facet> {
	public:
		Politope(const V* A, const V* B, const  V* C, const V* D, const size_t& A_id, const size_t& B_id, const size_t& C_id, const size_t& D_id ) : Hull<Facet>(A, B, C, D, A_id, B_id, C_id, D_id) {};
		std::list<Facet>* get_facets() { return &this->Facets;  };
		void Update_Hull(Facet* starting_facet_for_expansion, std::list<Facet*>* cone) { this->Hull<Facet>::__Update_Hull(starting_facet_for_expansion->Furthest_vertex, starting_facet_for_expansion, starting_facet_for_expansion->Furthest_Pos, cone); };
	};

	void __Compute_new_Convex_Hull_Master_thread(const size_t& max_iterations);
	void __Recompute_farthest(Facet* involved_face);
	static void ____distance_to_point(float* result, const V* vertex, const V* A);
	static void ____distance_to_line(float* result, const V* vertex, const V* A, const V* B);
#if defined(_OPENMP)
	void __Recompute_farthest_thread(const int& th_id);
#endif

// data
	const std::list<V>*				Vertices;
	Politope*								politope;
#if defined(_OPENMP)
	struct Farthest_info {
		const V*								local_Furthest_vertex;
		size_t									local_Furthest_Pos;
		float										local_Dist_Furthest_vertex;
	};
	int											Threads_number;
	bool										Threads_life;
	const Facet*							Thread_Involved_facet;
	std::list<Farthest_info>		Farthest_found;
	std::list<bool>						Thrad_life_local_copy;
#endif
};


///////////////////////////////////////////////////////
//							Implementations									 //
///////////////////////////////////////////////////////

template<typename V>
void Fast_QHull<V>::__Recompute_farthest(Facet* involved_face) {

#if defined(_OPENMP)

	this->Thread_Involved_facet = involved_face;
	this->__Recompute_farthest_thread(0);
	//do reduction
	auto it_far = this->Farthest_found.begin();
	Farthest_info* best = &(*it_far);
	it_far++;
	for (it_far; it_far != this->Farthest_found.end(); it_far++) {
		if (it_far->local_Furthest_vertex != NULL) {
			if (it_far->local_Dist_Furthest_vertex > best->local_Dist_Furthest_vertex)
				best = &(*it_far);
		}
	}
	involved_face->Dist_Furthest_vertex = best->local_Dist_Furthest_vertex;
	involved_face->Furthest_vertex = best->local_Furthest_vertex;
	involved_face->Furthest_Pos = best->local_Furthest_Pos;

#else

	auto it = this->Vertices->begin();
	involved_face->Furthest_vertex = &(*it);
	involved_face->Furthest_Pos = 0;
	involved_face->Dist_Furthest_vertex = (it->x() - involved_face->A->x()) * involved_face->N[0] + (it->y() - involved_face->A->y()) * involved_face->N[1] + (it->z() - involved_face->A->z()) * involved_face->N[2];
	it++;
	size_t k = 1;
	float distance;
	auto V_end = Vertices->end();
	for (it; it != V_end; it++) {
		distance = (it->x() - involved_face->A->x()) * involved_face->N[0] + (it->y() - involved_face->A->y()) * involved_face->N[1] + (it->z() - involved_face->A->z()) * involved_face->N[2];
		if (distance > involved_face->Dist_Furthest_vertex) {
			involved_face->Dist_Furthest_vertex = distance;
			involved_face->Furthest_vertex = &(*it);
			involved_face->Furthest_Pos = k;
		}
		k++;
	}

#endif

}

#if defined(_OPENMP)
template<typename V>
void Fast_QHull<V>::__Recompute_farthest_thread(const int& th_id) {

#pragma omp barrier
	if (!this->Threads_life) {
		auto it_life = this->Thrad_life_local_copy.begin();
		advance(it_life, th_id);
		*it_life = false;
		return;
	}

	auto it_far = this->Farthest_found.begin();
	advance(it_far, th_id);
	it_far->local_Furthest_vertex = NULL;

	if ((th_id + 1) > this->Vertices->size())
		return;

	auto it = this->Vertices->begin();
	advance(it, th_id);

	it_far->local_Furthest_vertex = &(*it);
	it_far->local_Furthest_Pos = th_id;
	it_far->local_Dist_Furthest_vertex = (it->x() - this->Thread_Involved_facet->A->x()) * this->Thread_Involved_facet->N[0] + (it->y() - this->Thread_Involved_facet->A->y()) * this->Thread_Involved_facet->N[1] + (it->z() - this->Thread_Involved_facet->A->z()) * this->Thread_Involved_facet->N[2];

	size_t k = th_id + this->Threads_number;
	size_t K_max = this->Vertices->size();
	if (k >= K_max) return;
	advance(it, this->Threads_number);
	float distance;
	auto V_end = Vertices->end();
	while (it != V_end) {
		distance = (it->x() - this->Thread_Involved_facet->A->x()) * this->Thread_Involved_facet->N[0] + (it->y() - this->Thread_Involved_facet->A->y()) * this->Thread_Involved_facet->N[1] + (it->z() - this->Thread_Involved_facet->A->z()) * this->Thread_Involved_facet->N[2];
		if (distance > it_far->local_Dist_Furthest_vertex) {
			it_far->local_Dist_Furthest_vertex = distance;
			it_far->local_Furthest_vertex = &(*it);
			it_far->local_Furthest_Pos = k;
		}
		k += this->Threads_number;
		if (k >= K_max) break;
		advance(it, this->Threads_number);
	}
#pragma omp barrier

}
#endif

template<typename V>
void Fast_QHull<V>::__Compute_new_Convex_Hull_Master_thread(const size_t& max_iterations) {

	//Start with building an initial non zero volume thetraedron. 
	const V* far_1, *far_2, *far_3, *far_4;
	size_t pos_1, pos_2, pos_3, pos_4;

	float dist_max, dist;
	size_t p;

	//first point is farthest along x axis
	auto it_V = this->Vertices->begin();
	dist_max = it_V->x();
	far_1 = &(*it_V);
	pos_1 = 0;
	it_V++;
	p = 1;
	auto V_end = this->Vertices->end();
	for (it_V; it_V != V_end; it_V++) {
		if (it_V->x() > dist_max) {
			dist_max = it_V->x();
			far_1 = &(*it_V);
			pos_1 = p;
		}
		p++;
	}

	//the second point considered will be the farthest point from the point P1
	it_V = this->Vertices->begin();
	this->____distance_to_point(&dist_max, &(*it_V), far_1);
	far_2 = &(*it_V);
	pos_2 = 0;
	it_V++;
	p = 1;
	for (it_V; it_V != V_end; it_V++) {
		this->____distance_to_point(&dist, &(*it_V), far_1);
		if (dist > dist_max) {
			dist_max = dist;
			far_2 = &(*it_V);
			pos_2 = p;
		}
		p++;
	}

	//the third point considered will be the farthest point of the Cloud from the line P1P2
	it_V = this->Vertices->begin();
	this->____distance_to_line(&dist_max, &(*it_V), far_2, far_1);
	far_3 = &(*it_V);
	pos_3 = 0;
	it_V++;
	p = 1;
	for (it_V; it_V != V_end; it_V++) {
		this->____distance_to_line(&dist, &(*it_V), far_2, far_1);
		if (dist > dist_max) {
			dist_max = dist;
			far_3 = &(*it_V);
			pos_3 = p;
		}
		p++;
	}

	float V1[3];
	V1[0] = far_1->x() - far_3->x();
	V1[1] = far_1->y() - far_3->y();
	V1[2] = far_1->z() - far_3->z();
	float V2[3];
	V2[0] = far_2->x() - far_3->x();
	V2[1] = far_2->y() - far_3->y();
	V2[2] = far_2->z() - far_3->z();
	float N[3];
	N[0] = V1[1] * V2[2] - V1[2] * V2[1];
	N[1] = V1[2] * V2[0] - V1[0] * V2[2];
	N[2] = V1[0] * V2[1] - V1[1] * V2[0];

	//the fourth point will be the farthest point of the Cloud, from the plane P1P2P3
	it_V = this->Vertices->begin();
	dist_max = abs(N[0] * (it_V->x() - far_1->x()) + N[1] * (it_V->y() - far_1->y()) + N[2] * (it_V->z() - far_1->z()));
	far_4 = &(*it_V);
	pos_4 = 0;
	it_V++;
	p = 1;
	for (it_V; it_V != V_end; it_V++) {
		dist = abs(N[0] * (it_V->x() - far_1->x()) + N[1] * (it_V->y() - far_1->y()) + N[2] * (it_V->z() - far_1->z()));
		if (dist > dist_max) {
			dist_max = dist;
			far_4 = &(*it_V);
			pos_4 = p;
		}
		p++;
	}

	this->politope = NULL;
	try {
		this->politope = new Politope(far_1, far_2, far_3, far_4, pos_1, pos_2, pos_3, pos_4);
	}
	catch (int ) {
		delete this->politope;
		this->politope = NULL;
		return;
	}

	for (auto it = this->politope->get_facets()->begin(); it != this->politope->get_facets()->end(); it++) {
		this->__Recompute_farthest(&(*it));
		if (it->Dist_Furthest_vertex <= GEOMETRIC_TOLLERANCE)
			it->Furthest_vertex = NULL;

		if ((it->Furthest_vertex == it->A) || (it->Furthest_vertex == it->B) || (it->Furthest_vertex == it->C))
			it->Furthest_vertex = NULL;
	}


	size_t Iterations_to_adopt, k_iter = 4;
	if (max_iterations == 0)
		Iterations_to_adopt = 0;
	else if (max_iterations <= 4)
		return;
	else
		Iterations_to_adopt = max_iterations - 4;

	//expand the initial thetraedron. Stop when there are no more facet expandable
	auto Facets = this->politope->get_facets();
	Facet* f_to_expand;
	auto it_facet = Facets->begin();
	std::list<Facet*> cone_of_new_facets;
	auto it_cone = cone_of_new_facets.begin();
	auto Facets_end = Facets->end();
	auto cone_end = cone_of_new_facets.end();
	while (true) {
		f_to_expand = NULL;
		it_facet = Facets->begin();
		Facets_end = Facets->end();
		for (it_facet; it_facet != Facets_end; it_facet++) {
			if (it_facet->Furthest_vertex != NULL) {
				f_to_expand = &(*it_facet);
				break;
			}
		}
		if (f_to_expand == NULL) break;

		for (it_facet; it_facet != Facets_end; it_facet++) {
			if (it_facet->Furthest_vertex != NULL) {
				if (it_facet->Dist_Furthest_vertex > f_to_expand->Dist_Furthest_vertex)
					f_to_expand = &(*it_facet);
			}
		}

		this->politope->Update_Hull(f_to_expand, &cone_of_new_facets);
		cone_end = cone_of_new_facets.end();
		for (it_cone = cone_of_new_facets.begin(); it_cone != cone_end; it_cone++) {
			this->__Recompute_farthest(*it_cone);
			if ((*it_cone)->Dist_Furthest_vertex <= GEOMETRIC_TOLLERANCE)
				(*it_cone)->Furthest_vertex = NULL;

			if (((*it_cone)->Furthest_vertex == (*it_cone)->A) || ((*it_cone)->Furthest_vertex == (*it_cone)->B) || ((*it_cone)->Furthest_vertex == (*it_cone)->C))
				(*it_cone)->Furthest_vertex = NULL;
		}

		k_iter++;
		if (k_iter == Iterations_to_adopt) break;
	}

}

template<typename V>
void Fast_QHull<V>::Compute_new_Convex_Hull(const std::list<V>* new_set_of_vertices, const size_t& max_iterations) {

	if (this->politope != NULL) delete this->politope;
	this->politope = NULL;
	this->Vertices = NULL;

	if (new_set_of_vertices->size() < 4) 
		return;

	this->Vertices = new_set_of_vertices;

#if defined(_OPENMP)
	this->Threads_number = omp_get_num_procs();
	this->Threads_life = true;
	this->Farthest_found.clear();
	this->Thrad_life_local_copy.clear();
	for (size_t k = 0; k < this->Threads_number; k++) {
		this->Farthest_found.push_back(Farthest_info());
		this->Thrad_life_local_copy.push_back(true);
	}

#pragma omp parallel \
num_threads(this->Threads_number)
	{
		int th_id = omp_get_thread_num();
		if (th_id == 0) {

			this->__Compute_new_Convex_Hull_Master_thread(max_iterations);

			this->Threads_life = false;
#pragma omp barrier
		}
		else {
			auto it_life = this->Thrad_life_local_copy.begin();
			advance(it_life, th_id);
			while (true) {
				this->__Recompute_farthest_thread(th_id);
				if (!(*it_life)) break;
			}

		}

	}

#else

	this->__Compute_new_Convex_Hull_Master_thread(max_iterations);

#endif

}

template<typename V>
void Fast_QHull<V>::Get_Incidences(std::list<std::list<size_t>>* indexes)  const {

	indexes->clear();
	if (this->politope == NULL) {
		return;
	}

	auto Facets = this->politope->get_facets();
	for (auto it = Facets->begin(); it != Facets->end(); it++)
		indexes->push_back({ it->id_A , it->id_B , it->id_C });

}

template<typename V>
size_t* Fast_QHull<V>::Get_Incidences(size_t* numb_of_facets)  const {

	if (this->politope == NULL) {
		return NULL;
	}

	auto Facets = this->politope->get_facets();
	size_t* indexes = (size_t*)malloc(sizeof(size_t)*Facets->size() * 3);

	*numb_of_facets = Facets->size();
	size_t k = 0;
	for (auto it = Facets->begin(); it != Facets->end(); it++) {
		indexes[k] = it->id_A;
		indexes[k + 1] = it->id_B;
		indexes[k + 2] = it->id_C;
		k += 3;
	}
	return indexes;

}

template<typename V>
void Fast_QHull<V>::Get_Normals(std::list<V>* normals)  const {

	normals->clear();
	if (this->politope == NULL) {
		return;
	}

	auto Facets = this->politope->get_facets();
	for (auto it = Facets->begin(); it != Facets->end(); it++)
		normals->push_back(V(it->N[0], it->N[1], it->N[2]));

}

template<typename V>
void Fast_QHull<V>::____distance_to_point(float* result, const V* vertex, const V* A) {

	*result = (vertex->x() - A->x()) * (vertex->x() - A->x());
	*result += (vertex->y() - A->y()) * (vertex->y() - A->y());
	*result += (vertex->z() - A->z()) * (vertex->z() - A->z());

};

template<typename V>
void Fast_QHull<V>::____distance_to_line(float* result, const V* vertex, const V* A, const V* B) {

	float V1[3];
	V1[0] = A->x() - vertex->x();
	V1[1] = A->y() - vertex->y();
	V1[2] = A->z() - vertex->z();
	float V2[3];
	V2[0] = B->x() - A->x();
	V2[1] = B->y() - A->y();
	V2[2] = B->z() - A->z();
	float s_opt = -(V1[0] * V2[0] + V1[1] * V2[1] + V1[2] * V2[2]) / (V2[0] * V2[0] + V2[1] * V2[1] + V2[2] * V2[2]);

	*result = 2.f * s_opt *(V1[0] * V2[0] + V1[1] * V2[1] + V1[2] * V2[2]);
	*result += s_opt * s_opt*(V2[0] * V2[0] + V2[1] * V2[1] + V2[2] * V2[2]);
	*result += (V1[0] * V1[0] + V1[1] * V1[1] + V1[2] * V1[2]);

}

#endif 
