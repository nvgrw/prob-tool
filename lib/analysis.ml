
(* module Mat = Owl.Sparse.Matrix.D *)
module Mat = Owl.Mat

let con = Llvm.global_context ()

let is_void v =
  Llvm.type_of v == Llvm.void_type con

let labels = Hashtbl.create 100
let current_label = ref 0
let add_label_to instr =
  Hashtbl.add labels instr !current_label;
  incr current_label
let get_label_of instr =
  Hashtbl.find labels instr
let with_label instr func =
  try func (Hashtbl.find labels instr) with
  | Not_found -> ()
let first_labeled_instr_in block =
  let rec helper = function
    | Llvm.Before instr -> (
        try
          (get_label_of instr, instr)
        with Not_found -> helper (Llvm.instr_succ instr))
    | Llvm.At_end _ -> assert false
  in
  helper (Llvm.instr_begin block)
let is_labelable instr =
  begin match Llvm.instr_opcode instr with
    | Llvm.Opcode.Br
    | Llvm.Opcode.Choose
    | Llvm.Opcode.Switch
    | Llvm.Opcode.Store
    | Llvm.Opcode.Select
    | Llvm.Opcode.Ret
      -> true
    | _ -> false
  end

(* Labeling *)
let label_functions f =
  let label_instructions i =
    if (is_labelable i) then
      add_label_to i;
    if not (is_void i) then
      Llvm.set_value_name "" i;
    with_label i (fun l -> Printf.printf "%02d |" l);
    print_endline @@ Llvm.string_of_llvalue i
  in
  let label_blocks b =
    Printf.printf "%%%s:\n" (Llvm.value_name @@ Llvm.value_of_block b);
    Llvm.iter_instrs label_instructions b
  in
  Printf.printf "(%s)\n" (Llvm.value_name f);
  Llvm.iter_blocks label_blocks f

(* Edges *)
let edge_functions f =
  let edges = Mat.zeros !current_label !current_label in
  let edge_instructions i =
    with_label i begin fun label ->
      if (Llvm.is_terminator i) then
        match Llvm.num_successors i with
        | 0 -> Mat.set edges label label 1.0
        | _ ->
          Llvm.iter_successors begin fun b ->
            let (succ_label, _instr) = first_labeled_instr_in b in
            Mat.set edges label succ_label 1.0
          end i
      else
        Mat.set edges label (label + 1) 1.0
    end
  in
  let edge_blocks b = Llvm.iter_instrs edge_instructions b in
  Llvm.iter_blocks edge_blocks f;
  edges

let mat_project _dim _vars _expr _var = ()

(* Translation *)
let translate_functions f =
  let variables = Llvm.fold_left_blocks (fun vs b ->
      (Llvm.fold_left_instrs (fun vs i ->
           match Llvm.instr_opcode i with
           | Llvm.Opcode.Alloca -> i :: vs
           | _ -> vs
         ) [] b) @ vs) [] f in
  let num_variables = List.length variables in
  let dimension = 3 * num_variables in
  Printf.printf "%d variables\n" num_variables;
  let translate_instrs mats i = let _mat = Mat.zeros dimension dimension in
    match Llvm.instr_opcode i with
    | Llvm.Opcode.Store ->
      print_endline @@ Llvm.string_of_llvalue @@ Llvm.operand i 1;
      mats
    | _ -> mats
  in
  Llvm.fold_left_blocks (fun mats b -> (
        Llvm.fold_left_instrs (fun mats i ->
            translate_instrs mats i) [] b) @ mats) [] f

let analyze mdl =
  let pass_manager = Llvm.PassManager.create () in
  (* passes *)
  Llvm_scalar_opts.add_cfg_simplification pass_manager;
  Llvm_scalar_opts.add_instruction_combination pass_manager;
  Llvm_scalar_opts.add_constant_propagation pass_manager;
  (* Llvm_scalar_opts.add_memory_to_register_promotion pass_manager; *)
  Llvm_scalar_opts.add_memory_to_register_demotion pass_manager;
  Llvm_scalar_opts.add_merged_load_store_motion pass_manager;
  Llvm_scalar_opts.add_dce pass_manager;
  (* end passes *)
  ignore @@ Llvm.PassManager.run_module mdl pass_manager;
  Llvm.PassManager.dispose pass_manager;

  (* remove declarations *)
  Llvm.iter_functions begin fun f ->
    if (Llvm.is_declaration f) then () else (
      print_endline "<< Labeling >>";
      label_functions f;
      print_endline "<< Edge Generation >>";
      let _edge_matrix = edge_functions f in
      print_endline "<< Translation >>";
      let translate_matrices = translate_functions f in
      List.iter (fun mat ->
          print_endline "::: ::: ::: ::: :::";
          Mat.print ~header:false mat;
          Format.pp_print_flush Format.std_formatter ();
          print_endline "||| ||| ||| ||| |||";
        ) translate_matrices
    )
  end mdl;
  ()