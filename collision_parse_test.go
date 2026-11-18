package diffik

import (
	"encoding/json"
	"reflect"
	"testing"
)

const binWorldState = `{
  "obstacles": [
    {"reference_frame": "world", "geometries": [
      {"center": {"x": 450, "y": -350, "z": 40, "o_z": 1}, "box": {"dims_mm": {"x": 152, "y": 152, "z": 82}}, "label": "bin"}
    ]}
  ]
}`

func TestParseWorldState(t *testing.T) {
	var obj map[string]interface{}
	if err := json.Unmarshal([]byte(binWorldState), &obj); err != nil {
		t.Fatal(err)
	}
	for _, in := range []interface{}{obj, []byte(binWorldState)} {
		ws, err := ParseWorldState(in)
		if err != nil {
			t.Fatal(err)
		}
		obs := ws.Obstacles()
		if len(obs) != 1 || len(obs[0].Geometries()) != 1 || obs[0].Geometries()[0].Label() != "bin" {
			t.Fatalf("obstacles = %+v", obs)
		}
	}
	if _, err := ParseWorldState(map[string]interface{}{"obstacle": []interface{}{}}); err == nil {
		t.Fatal("unknown field must be an error")
	}
}

func TestParseCollisionSpecs(t *testing.T) {
	var v interface{}
	js := `[{"allows":[{"frame1":"a","frame2":"b"}]},{"allows":[{"frame1":"c","frame2":"d"}]}]`
	if err := json.Unmarshal([]byte(js), &v); err != nil {
		t.Fatal(err)
	}
	got, err := ParseCollisionSpecs(v)
	if err != nil {
		t.Fatal(err)
	}
	if want := [][2]string{{"a", "b"}, {"c", "d"}}; !reflect.DeepEqual(got, want) {
		t.Fatalf("got %v, want %v", got, want)
	}
	if err := json.Unmarshal([]byte(`[{"allows":[{"frame1":"","frame2":"b"}]}]`), &v); err != nil {
		t.Fatal(err)
	}
	if _, err := ParseCollisionSpecs(v); err == nil {
		t.Fatal("empty frame1 must be an error")
	}
}

func TestParseStrings(t *testing.T) {
	got, err := ParseCollisionSpecs(`[{"allows":[{"frame1":"a","frame2":"b"}]}]`)
	if err != nil || !reflect.DeepEqual(got, [][2]string{{"a", "b"}}) {
		t.Fatalf("specs string: %v, %v", got, err)
	}
	if _, err := ParseCollisionSpecs(`{`); err == nil {
		t.Fatal("bad specs string must error")
	}
	ws, err := ParseWorldState(`{"obstacles":[]}`)
	if err != nil || ws == nil {
		t.Fatalf("world_state string: %v, %v", ws, err)
	}
	if _, err := ParseWorldState(`{"bogus":1}`); err == nil {
		t.Fatal("bad world_state string must error")
	}
}
