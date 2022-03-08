chmod +x debian/rules
fakeroot debian/rules clean
fakeroot debian/rules build
fakeroot debian/rules binary
