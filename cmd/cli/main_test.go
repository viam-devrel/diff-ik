package main

import "testing"

func TestParseJoints(t *testing.T) {
	lim := so101(t).DoF()
	for _, c := range []struct {
		in string
		ok bool
	}{
		{"0,-0.61,0.46,1.39,0", true},
		{"0,-0.61,0.46,1.39", false},
		{"0,NaN,0.46,1.39,0", false},
		{"0,Inf,0.46,1.39,0", false},
		{"0,-0.61,0.46,1.39,100", false},
		{"0,x,0.46,1.39,0", false},
	} {
		if _, err := parseJoints(c.in, lim); (err == nil) != c.ok {
			t.Errorf("%q: err %v, want ok=%v", c.in, err, c.ok)
		}
	}
}
