# navmesh-parser
 
A parser for navmesh version 16, which is currently used by CS:GO.
This code is mainly targeted towards pathfinding.

## Getting started

Just copy the files into your project and you're ready to go!

## Example
```cpp
try {
  nav_mesh::nav_file map_nav( "path/to/map.nav" ); 
  //Alternatively, you can just call map_nav.load( "path/to/map.nav" );
 
  //Figure out from where to where you'd like to find a path
 
  auto path = map_nav.find_path( start_point, end_point );
 
  if ( !path.empty( ) ) {
   //Do something
  }
} catch ( const std::exception& e ) {
  std::cout << e.what( ) << std::endl;
}
```

## Todo

- Parse Ladders properly

## Credits

- Valve ([Source Engine SDK 2013](https://github.com/ValveSoftware/source-sdk-2013)), code for parsing
- Lee Thomason ([Micropather](https://github.com/leethomason/MicroPather))
- Matthew Razza ([gonav](https://github.com/mrazza/gonav)), unknown data at end of each Area
