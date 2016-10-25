
// Imported from RF/trace/common.h ////////////////////////////////////////////
static inline int rfResolveOriginIndex(rfOriginTable *origintable,
                                       const rff *origin)
{
  int x, y, z;

  if( origin[RF_AXIS_X] < origintable->graphedge[RF_EDGE_MINX] )
    x = 0;
  else if( origin[RF_AXIS_X] > origintable->graphedge[RF_EDGE_MAXX] )
    x = origintable->max;
  else
  {
    x = (int)rffloor( origintable->spacinginv[RF_AXIS_X] * ( origin[RF_AXIS_X] - origintable->edge[RF_EDGE_MINX] ) );
    if( x < 0 )
      x = 0;
    else if( x > origintable->max )
      x = origintable->max;
  }

  if( origin[RF_AXIS_Y] < origintable->graphedge[RF_EDGE_MINY] )
    y = 0;
  else if( origin[RF_AXIS_Y] > origintable->graphedge[RF_EDGE_MAXY] )
    y = origintable->max;
  else
  {
    y = (int)rffloor( origintable->spacinginv[RF_AXIS_Y] * ( origin[RF_AXIS_Y] - origintable->edge[RF_EDGE_MINY] ) );
    if( y < 0 )
      y = 0;
    else if( y > origintable->max )
      y = origintable->max;
  }

  if( origin[RF_AXIS_Z] < origintable->graphedge[RF_EDGE_MINZ] )
    z = 0;
  else if( origin[RF_AXIS_Z] > origintable->graphedge[RF_EDGE_MAXZ] )
    z = origintable->max;
  else
  {
    z = (int)rffloor( origintable->spacinginv[RF_AXIS_Z] * ( origin[RF_AXIS_Z] - origintable->edge[RF_EDGE_MINZ] ) );
    if( z < 0 )
      z = 0;
    else if( z > origintable->max )
      z = origintable->max;
  }

  return ( x + ( y * origintable->factory ) + ( z * origintable->factorz ) );
}
///////////////////////////////////////////////////////////////////////////////

static inline void *resolve( void *graphmemory, const rff *origin )
{
  rfssize slink;
  int linkflags, nodeside;
  int index;
  rfOriginTable *origintable;
  void *root;

  origintable = (rfOriginTable *)RF_ADDRESS( graphmemory, RF_GRAPHHEADER_SIZE );
  index = rfResolveOriginIndex( origintable, origin );
  root = RF_ADDRESS( graphmemory, RF_ORIGINTABLE_LIST(origintable)[index] );

/*
printf( "Index %d, edge %f,%f ; %f,%f ; %f,%f\n", (int)index, RF_SECTOR(root)->edge[0], RF_SECTOR(root)->edge[1], RF_SECTOR(root)->edge[2], RF_SECTOR(root)->edge[3], RF_SECTOR(root)->edge[4], RF_SECTOR(root)->edge[5] );
*/

  /* We only need to walk "up" on XYZ */

  for( ; ; )
  {
    if( origin[RF_AXIS_X] < RF_SECTOR(root)->edge[ RF_EDGE_MAXX ] )
      break;
    if( !( RF_SECTOR(root)->flags & ( (RF_LINK_SECTOR<<RF_SECTOR_LINKFLAGS_SHIFT) << RF_EDGE_MAXX ) ) )
    {
      root = RF_ADDRESS( root, (rfssize)RF_SECTOR(root)->link[ RF_EDGE_MAXX ] << RF_LINKSHIFT );
      for( ; ; )
      {
        linkflags = RF_NODE(root)->flags;
        nodeside = RF_NODE_MORE;
        if( origin[ RF_NODE_GET_AXIS(linkflags) ] < RF_NODE(root)->plane )
          nodeside = RF_NODE_LESS;
        root = RF_ADDRESS( root, (rfssize)RF_NODE(root)->link[nodeside] << RF_LINKSHIFT );
        if( linkflags & ( (RF_LINK_SECTOR<<RF_NODE_LINKFLAGS_SHIFT) << nodeside ) )
          break;
      }
    }
    else
    {
      slink = (rfssize)RF_SECTOR(root)->link[ RF_EDGE_MAXX ];
      if( !( slink ) )
        break;
      root = RF_ADDRESS( root, slink << RF_LINKSHIFT );
    }
  }

  for( ; ; )
  {
    if( origin[RF_AXIS_Y] < RF_SECTOR(root)->edge[ RF_EDGE_MAXY ] )
      break;
    if( !( RF_SECTOR(root)->flags & ( (RF_LINK_SECTOR<<RF_SECTOR_LINKFLAGS_SHIFT) << RF_EDGE_MAXY ) ) )
    {
      root = RF_ADDRESS( root, (rfssize)RF_SECTOR(root)->link[ RF_EDGE_MAXY ] << RF_LINKSHIFT );
      for( ; ; )
      {
        linkflags = RF_NODE(root)->flags;
        nodeside = RF_NODE_MORE;
        if( origin[ RF_NODE_GET_AXIS(linkflags) ] < RF_NODE(root)->plane )
          nodeside = RF_NODE_LESS;
        root = RF_ADDRESS( root, (rfssize)RF_NODE(root)->link[nodeside] << RF_LINKSHIFT );
        if( linkflags & ( (RF_LINK_SECTOR<<RF_NODE_LINKFLAGS_SHIFT) << nodeside ) )
          break;
      }
    }
    else
    {
      slink = (rfssize)RF_SECTOR(root)->link[ RF_EDGE_MAXY ];
      if( !( slink ) )
        break;
      root = RF_ADDRESS( root, slink << RF_LINKSHIFT );
    }
  }

  for( ; ; )
  {
    if( origin[RF_AXIS_Z] < RF_SECTOR(root)->edge[ RF_EDGE_MAXZ ] )
      break;
    if( !( RF_SECTOR(root)->flags & ( (RF_LINK_SECTOR<<RF_SECTOR_LINKFLAGS_SHIFT) << RF_EDGE_MAXZ ) ) )
    {
      root = RF_ADDRESS( root, (rfssize)RF_SECTOR(root)->link[ RF_EDGE_MAXZ ] << RF_LINKSHIFT );
      for( ; ; )
      {
        linkflags = RF_NODE(root)->flags;
        nodeside = RF_NODE_MORE;
        if( origin[ RF_NODE_GET_AXIS(linkflags) ] < RF_NODE(root)->plane )
          nodeside = RF_NODE_LESS;
        root = RF_ADDRESS( root, (rfssize)RF_NODE(root)->link[nodeside] << RF_LINKSHIFT );
        if( linkflags & ( (RF_LINK_SECTOR<<RF_NODE_LINKFLAGS_SHIFT) << nodeside ) )
          break;
      }
    }
    else
    {
      slink = (rfssize)RF_SECTOR(root)->link[ RF_EDGE_MAXZ ];
      if( !( slink ) )
        break;
      root = RF_ADDRESS( root, slink << RF_LINKSHIFT );
    }
  }

  return root;
}
