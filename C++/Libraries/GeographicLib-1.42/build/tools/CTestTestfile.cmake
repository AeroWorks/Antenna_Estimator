# CMake generated Testfile for 
# Source directory: /Users/Maverick/Documents/GeographicLib-1.42/tools
# Build directory: /Users/Maverick/Documents/GeographicLib-1.42/build/tools
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(GeoConvert0 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/GeoConvert" "-p" "-3" "-m" "--input-string" "33.3 44.4")
set_tests_properties(GeoConvert0 PROPERTIES  PASS_REGULAR_EXPRESSION "38SMB4484")
add_test(GeoConvert1 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/GeoConvert" "-d" "--input-string" "38smb")
set_tests_properties(GeoConvert1 PROPERTIES  PASS_REGULAR_EXPRESSION "32d59'14\\.1\"N 044d27'53\\.4\"E")
add_test(GeoConvert2 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/GeoConvert" "-p" "-2" "--input-string" "30d30'30\" 30.50833")
set_tests_properties(GeoConvert2 PROPERTIES  PASS_REGULAR_EXPRESSION "30\\.508 30\\.508")
add_test(GeoConvert3 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/GeoConvert" "--junk")
set_tests_properties(GeoConvert3 PROPERTIES  WILL_FAIL "ON")
add_test(GeoConvert4 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/GeoConvert" "--input-string" "garbage")
set_tests_properties(GeoConvert4 PROPERTIES  WILL_FAIL "ON")
add_test(GeoConvert5 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/GeoConvert" "--input-string" "5d. 0")
set_tests_properties(GeoConvert5 PROPERTIES  WILL_FAIL "ON")
add_test(GeoConvert6 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/GeoConvert" "-p" "9" "--input-string" "0 179.99999999999998578")
set_tests_properties(GeoConvert6 PROPERTIES  PASS_REGULAR_EXPRESSION "179\\.9999999999999[7-9]")
add_test(GeodSolve0 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/GeodSolve" "-i" "-p" "0" "--input-string" "40.6 -73.8 49d01'N 2d33'E")
set_tests_properties(GeodSolve0 PROPERTIES  PASS_REGULAR_EXPRESSION "53\\.47022 111\\.59367 5853226")
add_test(GeodSolve1 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/GeodSolve" "-p" "0" "--input-string" "40d38'23\"N 073d46'44\"W 53d30' 5850e3")
set_tests_properties(GeodSolve1 PROPERTIES  PASS_REGULAR_EXPRESSION "49\\.01467 2\\.56106 111\\.62947")
add_test(GeodSolve2 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/GeodSolve" "-i" "-p" "0" "-e" "6.4e6" "-1/150" "--input-string" "0.07476 0 -0.07476 180")
set_tests_properties(GeodSolve2 PROPERTIES  PASS_REGULAR_EXPRESSION "90\\.00078 90\\.00078 20106193")
add_test(GeodSolve3 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/GeodSolve" "-i" "-p" "0" "-e" "6.4e6" "-1/150" "--input-string" "0.1 0 -0.1 180")
set_tests_properties(GeodSolve3 PROPERTIES  PASS_REGULAR_EXPRESSION "90\\.00105 90\\.00105 20106193")
add_test(GeodSolve4 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/GeodSolve" "-i" "--input-string" "36.493349428792 0 36.49334942879201 .0000008")
set_tests_properties(GeodSolve4 PROPERTIES  PASS_REGULAR_EXPRESSION ".* .* 0\\.072")
add_test(GeodSolve5 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/GeodSolve" "-p" "0" "--input-string" "0.01777745589997 30 0 10e6")
set_tests_properties(GeodSolve5 PROPERTIES  PASS_REGULAR_EXPRESSION "90\\.00000 -150\\.00000 -180\\.00000;90\\.00000 30\\.00000 0\\.00000")
add_test(GeodSolve6 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/GeodSolve" "-i" "--input-string" "88.202499451857 0 -88.202499451857 179.981022032992859592")
set_tests_properties(GeodSolve6 PROPERTIES  PASS_REGULAR_EXPRESSION ".* .* 20003898.214")
add_test(GeodSolve7 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/GeodSolve" "-i" "--input-string" "89.262080389218 0 -89.262080389218 179.992207982775375662")
set_tests_properties(GeodSolve7 PROPERTIES  PASS_REGULAR_EXPRESSION ".* .* 20003925.854")
add_test(GeodSolve8 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/GeodSolve" "-i" "--input-string" "89.333123580033 0 -89.333123580032997687 179.99295812360148422")
set_tests_properties(GeodSolve8 PROPERTIES  PASS_REGULAR_EXPRESSION ".* .* 20003926.881")
add_test(GeodSolve9 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/GeodSolve" "-i" "--input-string" "56.320923501171 0 -56.320923501171 179.664747671772880215")
set_tests_properties(GeodSolve9 PROPERTIES  PASS_REGULAR_EXPRESSION ".* .* 19993558.287")
add_test(GeodSolve10 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/GeodSolve" "-i" "--input-string" "52.784459512564 0 -52.784459512563990912 179.634407464943777557")
set_tests_properties(GeodSolve10 PROPERTIES  PASS_REGULAR_EXPRESSION ".* .* 19991596.095")
add_test(GeodSolve11 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/GeodSolve" "-i" "--input-string" "48.522876735459 0 -48.52287673545898293 179.599720456223079643")
set_tests_properties(GeodSolve11 PROPERTIES  PASS_REGULAR_EXPRESSION ".* .* 19989144.774")
add_test(GeodSolve12 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/GeodSolve" "-i" "-e" "89.8" "-1.83" "-p" "1" "--input-string" "0 0 -10 160")
set_tests_properties(GeodSolve12 PROPERTIES  PASS_REGULAR_EXPRESSION "120\\.27.* 105\\.15.* 266\\.7")
add_test(GeodSolve13 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/GeodSolve" "-i" "-e" "89.8" "-1.83" "-p" "1" "--input-string" "0 0 -10 160" "-E")
set_tests_properties(GeodSolve13 PROPERTIES  PASS_REGULAR_EXPRESSION "120\\.27.* 105\\.15.* 266\\.7")
add_test(GeodSolve14 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/GeodSolve" "-i" "--input-string" "0 0 1 nan")
set_tests_properties(GeodSolve14 PROPERTIES  PASS_REGULAR_EXPRESSION "nan nan nan")
add_test(GeodSolve15 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/GeodSolve" "-e" "6.4e6" "-1/150" "-f" "--input-string" "1 2 3 4")
set_tests_properties(GeodSolve15 PROPERTIES  PASS_REGULAR_EXPRESSION "1\\..* 2\\..* 3\\..* 1\\..* 2\\..* 3\\..* 4\\..* 0\\..* 4\\..* 1\\..* 1\\..* 23700")
add_test(GeodSolve16 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/GeodSolve" "-e" "6.4e6" "-1/150" "-f" "--input-string" "1 2 3 4" "-E")
set_tests_properties(GeodSolve16 PROPERTIES  PASS_REGULAR_EXPRESSION "1\\..* 2\\..* 3\\..* 1\\..* 2\\..* 3\\..* 4\\..* 0\\..* 4\\..* 1\\..* 1\\..* 23700")
add_test(Planimeter0 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/Planimeter" "--input-string" "89 0;89 90;89 180;89 270")
set_tests_properties(Planimeter0 PROPERTIES  PASS_REGULAR_EXPRESSION "4 631819\\.8745[0-9]+ 2495230567[78]\\.[0-9]+")
add_test(Planimeter1 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/Planimeter" "-r" "--input-string" "-89 0;-89 90;-89 180;-89 270")
set_tests_properties(Planimeter1 PROPERTIES  PASS_REGULAR_EXPRESSION "4 631819\\.8745[0-9]+ 2495230567[78]\\.[0-9]+")
add_test(Planimeter2 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/Planimeter" "--input-string" "0 -1;-1 0;0 1;1 0")
set_tests_properties(Planimeter2 PROPERTIES  PASS_REGULAR_EXPRESSION "4 627598\\.2731[0-9]+ 24619419146.[0-9]+")
add_test(Planimeter3 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/Planimeter" "--input-string" "90 0; 0 0; 0 90")
set_tests_properties(Planimeter3 PROPERTIES  PASS_REGULAR_EXPRESSION "3 30022685\\.[0-9]+ 63758202715511\\.[0-9]+")
add_test(Planimeter4 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/Planimeter" "-l" "--input-string" "90 0; 0 0; 0 90")
set_tests_properties(Planimeter4 PROPERTIES  PASS_REGULAR_EXPRESSION "3 20020719\\.[0-9]+")
add_test(Planimeter5 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/Planimeter" "--input-string" "89,0.1;89,90.1;89,-179.9")
set_tests_properties(Planimeter5 PROPERTIES  PASS_REGULAR_EXPRESSION "3 539297\\.[0-9]+ 1247615283[89]\\.[0-9]+")
add_test(Planimeter6 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/Planimeter" "-p" "8" "--input-string" "9 -0.00000000000001;9 180;9 0")
set_tests_properties(Planimeter6 PROPERTIES  PASS_REGULAR_EXPRESSION "3 36026861\\.[0-9]+ -?0.0[0-9]+")
add_test(Planimeter7 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/Planimeter" "-p" "8" "--input-string" "9  0.00000000000001;9 0;9 180")
set_tests_properties(Planimeter7 PROPERTIES  PASS_REGULAR_EXPRESSION "3 36026861\\.[0-9]+ -?0.0[0-9]+")
add_test(Planimeter8 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/Planimeter" "-p" "8" "--input-string" "9  0.00000000000001;9 180;9 0")
set_tests_properties(Planimeter8 PROPERTIES  PASS_REGULAR_EXPRESSION "3 36026861\\.[0-9]+ -?0.0[0-9]+")
add_test(Planimeter9 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/Planimeter" "-p" "8" "--input-string" "9 -0.00000000000001;9 0;9 180")
set_tests_properties(Planimeter9 PROPERTIES  PASS_REGULAR_EXPRESSION "3 36026861\\.[0-9]+ -?0.0[0-9]+")
add_test(Planimeter10 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/Planimeter" "-R" "--input-string" "41N 111:3W; 41N 104:3W; 45N 104:3W; 45N 111:3W")
set_tests_properties(Planimeter10 PROPERTIES  PASS_REGULAR_EXPRESSION "4 2029616\\.[0-9]+ 2535883763..\\.")
add_test(Planimeter11 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/Planimeter" "-R" "--input-string" "66:33:44 0; 66:33:44 180")
set_tests_properties(Planimeter11 PROPERTIES  PASS_REGULAR_EXPRESSION "2 15985058\\.[0-9]+ 212084182523..\\.")
add_test(Planimeter12 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/Planimeter" "--input-string" "66:33:44 0; 66:33:44 180")
set_tests_properties(Planimeter12 PROPERTIES  PASS_REGULAR_EXPRESSION "2 10465729\\.[0-9]+ -?0.0")
add_test(Planimeter13 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/Planimeter" "--input-string" "89 -360; 89 -240; 89 -120; 89 0; 89 120; 89 240")
set_tests_properties(Planimeter13 PROPERTIES  PASS_REGULAR_EXPRESSION "6 1160741\\..* 32415230256\\.")
add_test(ConicProj0 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/ConicProj" "-a" "40d58" "39d56" "-l" "77d45W" "-r" "--input-string" "220e3 -52e3")
set_tests_properties(ConicProj0 PROPERTIES  PASS_REGULAR_EXPRESSION "39\\.95[0-9]+ -75\\.17[0-9]+ 1\\.67[0-9]+ 0\\.99[0-9]+")
add_test(ConicProj1 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/ConicProj" "-a" "0" "0" "-e" "6.4e6" "-0.5" "-r" "--input-string" "0 8605508")
set_tests_properties(ConicProj1 PROPERTIES  PASS_REGULAR_EXPRESSION "^85\\.00")
add_test(ConicProj2 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/ConicProj" "-c" "-30" "-30" "--input-string" "-30 0")
set_tests_properties(ConicProj2 PROPERTIES  PASS_REGULAR_EXPRESSION "^-?0\\.0+ -?0\\.0+ -?0\\.0+ 1\\.0+")
add_test(ConicProj3 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/ConicProj" "-r" "-c" "0" "0" "--input-string" "1113195 -1e10")
set_tests_properties(ConicProj3 PROPERTIES  PASS_REGULAR_EXPRESSION "^-90\\.0+ 10\\.00[0-9]+ ")
add_test(ConicProj4 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/ConicProj" "-r" "-c" "0" "0" "--input-string" "1113195 inf")
set_tests_properties(ConicProj4 PROPERTIES  PASS_REGULAR_EXPRESSION "^90\\.0+ 10\\.00[0-9]+ ")
add_test(ConicProj5 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/ConicProj" "-r" "-c" "45" "45" "--input-string" "0 -1e100")
set_tests_properties(ConicProj5 PROPERTIES  PASS_REGULAR_EXPRESSION "^-90\\.0+ -?0\\.00[0-9]+ ")
add_test(ConicProj6 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/ConicProj" "-r" "-c" "45" "45" "--input-string" "0 -inf")
set_tests_properties(ConicProj6 PROPERTIES  PASS_REGULAR_EXPRESSION "^-90\\.0+ -?0\\.00[0-9]+ ")
add_test(ConicProj7 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/ConicProj" "-r" "-c" "90" "90" "--input-string" "0 -1e150")
set_tests_properties(ConicProj7 PROPERTIES  PASS_REGULAR_EXPRESSION "^-90\\.0+ -?0\\.00[0-9]+ ")
add_test(ConicProj8 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/ConicProj" "-r" "-c" "90" "90" "--input-string" "0 -inf")
set_tests_properties(ConicProj8 PROPERTIES  PASS_REGULAR_EXPRESSION "^-90\\.0+ -?0\\.00[0-9]+ ")
add_test(CartConvert0 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/CartConvert" "-e" "6.4e6" "1/100" "-r" "--input-string" "10e3 0 1e3")
set_tests_properties(CartConvert0 PROPERTIES  PASS_REGULAR_EXPRESSION "85\\.57[0-9]+ 0\\.0[0]+ -6334614\\.[0-9]+")
add_test(CartConvert1 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/CartConvert" "-e" "6.4e6" "-1/100" "-r" "--input-string" "1e3 0 10e3")
set_tests_properties(CartConvert1 PROPERTIES  PASS_REGULAR_EXPRESSION "4\\.42[0-9]+ 0\\.0[0]+ -6398614\\.[0-9]+")
add_test(TransverseMercatorProj0 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/TransverseMercatorProj" "-k" "1" "--input-string" "90 75")
set_tests_properties(TransverseMercatorProj0 PROPERTIES  PASS_REGULAR_EXPRESSION "^0\\.0+ 10001965\\.7293[0-9]+ 75\\.0+ 1\\.0+")
add_test(TransverseMercatorProj1 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/TransverseMercatorProj" "-k" "1" "-r" "--input-string" "0 10001965.7293127228")
set_tests_properties(TransverseMercatorProj1 PROPERTIES  PASS_REGULAR_EXPRESSION "(90\\.0+ 0\\.0+ 0\\.0+|(90\\.0+|89\\.99999999999[0-9]+) -?180\\.0+ -?180\\.0+) (1\\.0000+|0\\.9999+)")
add_test(RhumbSolve0 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/RhumbSolve" "-p" "3" "-i" "--input-string" "0 0 90 0")
set_tests_properties(RhumbSolve0 PROPERTIES  PASS_REGULAR_EXPRESSION "^0\\.0+ 10001965\\.729 ")
add_test(RhumbSolve1 "/Users/Maverick/Documents/GeographicLib-1.42/build/tools/RhumbSolve" "-p" "3" "-i" "--input-string" "0 0 90 0" "-s")
set_tests_properties(RhumbSolve1 PROPERTIES  PASS_REGULAR_EXPRESSION "^0\\.0+ 10001965\\.729 ")