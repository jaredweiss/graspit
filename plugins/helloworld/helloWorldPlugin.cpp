//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
//
// Author(s):  Matei T. Ciocarlie
//
// $Id: taskDispatcher.h,v 1.3 2010/04/12 20:15:30 cmatei Exp $
//
//######################################################################

#include "helloWorldPlugin.h"
#include <iostream>
#include <string>


namespace helloworld {

HelloWorldPlugin::HelloWorldPlugin()
{
  std::cerr << "Hello world plugin created\n";
}

HelloWorldPlugin::~HelloWorldPlugin()
{
  std::cerr << "Hello world plugin destroyed\n";
}

int HelloWorldPlugin::init(int argc, char** argv)
{
  std::cerr << "Hello world plugin initialized with args:\n";
  for(int i=0; i < argc; i++)
    {
      std::cerr << argv[i] << std::endl;
    }
  return 0;
}

int HelloWorldPlugin::mainLoop()
{
  std::cout << "hello world \n";
  return 0;
}

}



extern "C" PLUGIN_API Plugin* createPlugin() {
  return new helloworld::HelloWorldPlugin();
}


extern "C" PLUGIN_API std::string getType() {
  return "hello world";
}
