# Windtunnel
Hard-sphere particle simulation with dynamic collision detection

Simple application to simulate the behavior of gases in 2D or 3D using a hard-sphere simulation with dynamic collision detection.

Demonstrated in these videos:

https://www.youtube.com/watch?v=vwk4mSFFop0

https://www.youtube.com/watch?v=TyVx3UnZbVU

The application is based on the Vrui VR development toolkit, version 5.3-001 or later. Vrui is available from http://web.cs.ucdavis.edu/~okreylos/ResDev/Vrui/LinkDownload.html

Installation Guide
==================

It is recommended to download or move the source packages for Vrui and
the Windtunnel into a src directory underneath the user's home directory.
Otherwise, references to ~/src in the following instructions need to be
changed.

It is also recommended to skip optional steps 4 and 6 in the following
instructions. The Windtunnel does not need to be installed in order to be
used; installation (to a system directory such as /usr/local) is only
recommended if the Windtunnel will be used from multiple user accounts.

0. Install Vrui from ~/src/Vrui-&lt;version&gt;-&lt;build&gt; (see Vrui README file).

1. Change into ~/src directory and unpack the Windtunnel tarball:

   &gt; cd ~/src
   
   &gt; tar xfz &lt;download path&gt;/Windtunnel-&lt;version&gt;.tar.gz
   
   - or -
   
   &gt; tar xf &lt;download path&gt;/Windtunnel-&lt;version&gt;.tar

2. Change into the Windtunnel base directory:

   &gt; cd Windtunnel-&lt;version&gt;

3. If the Vrui version installed in step 0 was not 5.3, or Vrui's
   installation directory was changed from the default of /usr/local,
   adapt the makefile using a text editor. Change the value of
   VRUI_MAKEDIR close to the beginning of the file as follows:
   
   VRUI_MAKEDIR := &lt;Vrui install dir&gt;/share/make
   
   Where &lt;Vrui install dir&gt; is the installation directory chosen in
   step 0. Use $(HOME) to refer to the user's home directory instead
   of ~.

4. Optional: Adapt makefile if the Windtunnel is to be installed in a
   different location, for example /usr/local. Set INSTALLDIR to the
   desired target location. The Windtunnel will then be installed in
   &lt;INSTALLDIR&gt;/bin.

5. Build the Windtunnel:

   &gt; make
   
   This creates the CollisionBoxTest executable in ./bin.

6. Optional: Install the Windtunnel in the selected target location. This
   is only necessary if the INSTALLDIR variable in the makefile was
   changed. By default, the Windtunnel can be run from its base directory.
   
   To install:
   
   &gt; make install
   
   - or, if the target location is a system directory -
   
   &gt; sudo make install
   
   This will copy the executable into &lt;INSTALLDIR&gt;/bin.

7. Optional: Add directory containing the CollisionBoxTest executable
   (in ~/src/Windtunnel-&lt;version&gt;/bin in the default installation, in
   &lt;INSTALLDIR&gt;/bin otherwise) to the user's search path. This allows
   running the Windtunnel from any directory. Using csh or tcsh:
   
   &gt; setenv PATH ${PATH}:~/src/NCK-&lt;version&gt;/bin
   
   - or -
   
   &gt; setenv PATH ${PATH}:&lt;INSTALLDIR&gt;/bin
   
   where &lt;INSTALLDIR&gt; is the target location set in the makefile.
   
   Using bash:
   
   &gt; export PATH=${PATH}:~/src/NCK-&lt;version&gt;/bin
   
   - or -
   
   &gt; export PATH=${PATH}:&lt;INSTALLDIR&gt;/bin
   
   These lines can also be added to the user's .cshrc or .bashrc files
   to make the additions persist between logins.

Running the Windtunnel
======================

These instructions assume that the Windtunnel was installed in its base
directory (see steps 4 and 6 above).

1. Run the Windtunnel executable:

   &gt; cd ~/src/Windtunnel-&lt;version&gt;
   
   &gt; ./bin/CollisionBoxTest &lt;command line arguments&gt;

2. See Vrui's HTML documentation on Vrui's basic user interface and how
   to use the Windtunnel.
