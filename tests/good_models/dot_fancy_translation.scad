$fn=5;
union()
{
	translate([0,0,0])
	{
		rotate(45.000004,[0,0,1])
		{
			cube([1,1,1]);
		}
	}
	translate([-3,-4,-5])
	{
		rotate(45.000004,[0,0,1])
		{
			cube([1,1,1]);
		}
	}
}
