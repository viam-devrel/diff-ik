package diffik

import (
	"encoding/json"
	"fmt"

	commonpb "go.viam.com/api/common/v1"
	motionpb "go.viam.com/api/service/motion/v1"
	"go.viam.com/rdk/referenceframe"
	"google.golang.org/protobuf/encoding/protojson"
)

// ParseWorldState decodes a world_state value (a JSON object, or its bytes) in the motion service API's
// protojson shape (spec "Inputs"). Decoding is strict: unknown fields are errors.
func ParseWorldState(v interface{}) (*referenceframe.WorldState, error) {
	b, err := jsonBytes(v)
	if err != nil {
		return nil, fmt.Errorf("world_state: %w", err)
	}
	var pb commonpb.WorldState
	if err := protojson.Unmarshal(b, &pb); err != nil {
		return nil, fmt.Errorf("world_state: %w", err)
	}
	ws, err := referenceframe.WorldStateFromProtobuf(&pb)
	if err != nil {
		return nil, fmt.Errorf("world_state: %w", err)
	}
	return ws, nil
}

// ParseCollisionSpecs decodes a collision_specification list (motion.v1.CollisionSpecification each) into
// allowed (frame1, frame2) pairs.
func ParseCollisionSpecs(v interface{}) ([][2]string, error) {
	items, ok := v.([]interface{})
	if s, isStr := v.(string); isStr {
		var raws []json.RawMessage
		if err := json.Unmarshal([]byte(s), &raws); err != nil {
			return nil, fmt.Errorf("collision_specification: %w", err)
		}
		items, ok = make([]interface{}, len(raws)), true
		for i, r := range raws {
			items[i] = []byte(r)
		}
	}
	if !ok {
		return nil, fmt.Errorf("collision_specification must be a list, got %T", v)
	}
	var out [][2]string
	for i, it := range items {
		b, err := jsonBytes(it)
		if err != nil {
			return nil, fmt.Errorf("collision_specification[%d]: %w", i, err)
		}
		var pb motionpb.CollisionSpecification
		if err := protojson.Unmarshal(b, &pb); err != nil {
			return nil, fmt.Errorf("collision_specification[%d]: %w", i, err)
		}
		for _, a := range pb.GetAllows() {
			if a.GetFrame1() == "" || a.GetFrame2() == "" {
				return nil, fmt.Errorf("collision_specification[%d]: allows needs frame1 and frame2", i)
			}
			out = append(out, [2]string{a.GetFrame1(), a.GetFrame2()})
		}
	}
	return out, nil
}

func jsonBytes(v interface{}) ([]byte, error) {
	if b, ok := v.([]byte); ok {
		return b, nil
	}
	if s, ok := v.(string); ok { // protojson text, as teleop's decodeProto takes
		return []byte(s), nil
	}
	return json.Marshal(v)
}
