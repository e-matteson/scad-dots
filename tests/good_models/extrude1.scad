$fn=5;
translate([0,0,-5])
{
	linear_extrude(height=1,center=false,convecity=10,twist=0,slices=1)
	{
		polygon(points=[[-5,-5],[0,10],[20,10],],paths=undef,convexity=10);
	}
}
