package main

import (
	"diffik"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	motion "go.viam.com/rdk/services/motion"
)

func main() {
	// ModularMain can take multiple APIModel arguments, if your module implements multiple models.
	module.ModularMain(resource.APIModel{API: motion.API, Model: diffik.Servo})
}
