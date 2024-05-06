//definera utanför funktioner (ovanför main stabilize, globalt)
double int_part = 0;


// i main stabilize

int_part += rotz2;

if ( abs(int_part) > 3600 ) { int_part = 0; }


// lägg till ny updateVal funktion

void updateVal(float *inp_val, float req_val, float new_K, float new_I)
{
	float discr = (req_val - *inp_val);
	*inp_val += new_K*discr + new_I*int_part;
}
