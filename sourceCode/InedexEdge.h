#ifndef INDEXEDGE_H
#define INDEXEDGE_H

class InedexEdge
{
public:
	int _index1;
	int _index2;

	float _dist;
	float _oriDist;
	float _scale;


	InedexEdge()
	{
		_index1 = -1;
		_index2 = -1;

		_dist = 0.0f;
		_oriDist = 0.0f;
		_scale = 1.0f;
	}
	InedexEdge(int id1, int id2)
	{
		_index1 = id1;
		_index2 = id2;

		_dist = 0.0f;
		_oriDist = 0.0f;
		_scale = 1.0f;
	}
	InedexEdge(int id1, int id2, float dist)
	{
		_index1 = id1;
		_index2 = id2;

		_dist = dist;
		_oriDist = dist;
		_scale = 1.0f;
	}


	float GetDist() const { return _dist; }
	void SetDist(float d) { _dist = d; _oriDist = _dist; }

	void MakeLonger(float growth_scale_iter, float dt)
	{
		_scale += growth_scale_iter * dt;
		_dist = _oriDist * _scale;

	}

	void Swap()
	{
		_index1 ^= _index2;
		_index2 ^= _index1;
		_index1 ^= _index2;
	}
};
#endif // !EDGE_H
