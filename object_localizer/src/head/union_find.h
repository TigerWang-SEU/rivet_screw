#include <set>
#include <map>
#ifndef UNION_FIND_H
#define UNION_FIND_H

// define the union-find data structure
class UF
{
  int size, cnt, *id, *sz;

public:
  // Create an empty union find data structure with N isolated sets.
  UF ( int N )
  {
    size = N;
    cnt = N;
    id = new int[N];
    sz = new int[N];
    for ( int i = 0; i < N; i++ )
    {
      id[i] = i;
      sz[i] = 1;
    }
  }

  ~UF ()
  {
    delete [] id;
    delete [] sz;
  }

  // Return the id of component corresponding to object p.
  int find ( int p )
  {
    int root = p;
    while ( root != id [ root ] )
      root = id[root];
    while ( p != root )
    {
      int newp = id [ p ];
      id [ p ] = root;
      p = newp;
    }
    return root;
  }

  // Replace sets containing x and y with their union.
  void merge ( int x, int y )
  {
    int i = find ( x );
    int j = find ( y );
    if ( i == j ) return;

    // make smaller root point to larger one
    if ( sz [ i ] < sz [ j ] )
    {
      id [ i ] = j;
      sz [ j ] += sz [ i ];
    }
    else
    {
      id [ j ] = i;
      sz [ i ] += sz [ j ];
    }
    cnt--;
  }

  // Are objects x and y in the same set?
  bool connected ( int x, int y )
  {
    return find ( x ) == find ( y );
  }

  // Return the number of disjoint sets.
  int count()
  {
    return cnt;
  }

  // return the list of connected components
  std::map < int, std::list < int > > get_components ()
  {
    std::map < int, std::list < int > > component_map;
    for ( int i = 0; i < size; i++ )
    {
      int component_id = find ( i );
      if ( component_map.find ( component_id ) != component_map.end() )
      {
        component_map[ component_id ].push_back( i );
      }
      else
      {
        std::list < int > component = {i};
        component_map.insert ( std::pair< int, std::list < int > > ( component_id, component ) );
      }
    }
    return component_map;
  }
};

#endif
