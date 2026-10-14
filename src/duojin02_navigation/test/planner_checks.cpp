#include "duojin02_navigation/planner_checks.hpp"
#include "duojin02_navigation/finite_heading.hpp"
#include <cassert>
#include <iostream>
using namespace duojin02_navigation;

bool hardSweep(const std::vector<Pose> &path,const Polygon &polygon,
               const MapBounds &bounds,const std::vector<Point> &obstacles) {
  if(path.empty()) return false;
  double radius=0;
  for(const auto &p:polygon) radius=std::max(radius,std::hypot(p.x,p.y));
  auto valid=[&](const Pose &p) {
    return footprintInsideBounds(p,polygon,bounds,.002) && !collision(p,polygon,obstacles,.002);
  };
  if(!valid(path.front())) return false;
  for(size_t i=1;i<path.size();++i) {
    const auto &a=path[i-1],&b=path[i];
    const double turn=std::remainder(b.yaw-a.yaw,2*M_PI);
    const double dx=b.x-a.x,dy=b.y-a.y;
    const int steps=std::max(1,static_cast<int>(std::ceil((std::hypot(dx,dy)+radius*std::abs(turn))/.004)));
    for(int j=0;j<=steps;++j) {
      const double f=static_cast<double>(j)/steps;
      if(!valid({a.x+f*dx,a.y+f*dy,a.yaw+f*turn})) return false;
    }
  }
  return true;
}

void finiteHeadingTests() {
  const Polygon rectangle{{.3,.1},{-.3,.1},{-.3,-.1},{.3,-.1}};
  const MapBounds room{-1,-1,1,1};const double radius=std::hypot(.3,.1);
  auto hard=[&](const Pose &p) {return footprintInsideBounds(p,rectangle,room,.002) && !collision(p,rectangle,{},.002);};
  auto soft=[](const Pose &) {return static_cast<unsigned char>(176);};
  auto available=[](){return true;};
  const std::vector<Pose> ref{{-.2,0,0},{0,0,M_PI/2},{.2,0,M_PI/2},{.2,0,0}};
  const std::vector<std::vector<double>> options{{0},{0,0,M_PI/2,M_PI},{0,M_PI/2},{0}};
  const auto found=finiteHeadingSearch(ref,options,radius,hard,soft,available);
  assert(found.status==FiniteHeadingStatus::Found && found.path.size()==ref.size());
  for(size_t i=0;i<ref.size();++i) {
    assert(found.path[i].x==ref[i].x && found.path[i].y==ref[i].y);
    assert(found.path[i].yaw==0 && found.choices[i]==0);
  }
  assert(found.angular_travel==0);
  auto wrong_endpoints=options;wrong_endpoints.front()[0]=.01;
  assert(finiteHeadingSearch(ref,wrong_endpoints,radius,hard,soft,available).status==FiniteHeadingStatus::InvalidInput);
  auto wrong_xy=found.path;wrong_xy[1].x+=1e-12;
  assert(finiteHeadingVerify(ref,wrong_xy,radius,hard,soft,available).status==FiniteHeadingStatus::VerificationFailed);
  auto wrong_goal=found.path;wrong_goal.back().yaw=.01;
  assert(finiteHeadingVerify(ref,wrong_goal,radius,hard,soft,available).status==FiniteHeadingStatus::VerificationFailed);

  // An immutable source graph with safe nodes can still have no Q-valid edge.
  const std::vector<Pose> qref{{0,0,0},{.02,0,M_PI/2},{.04,0,0}};
  const std::vector<std::vector<double>> qoptions{{0},{0},{0}};
  auto one_more=[](const Pose &p) {return static_cast<unsigned char>(p.yaw==0?177:176);};
  const auto noQ=finiteHeadingSearch(qref,qoptions,radius,hard,one_more,available);
  assert(noQ.status==FiniteHeadingStatus::NoFeasible && noQ.counts.soft_rejected>0);
  assert(noQ.first_rejection.reason==FiniteHeadingFailure::StrictQ);

  // ds=0 excludes soft budget but its mid-turn still hits the filled corner pole.
  const std::vector<Pose> turn{{0,0,0},{0,0,M_PI/2}};
  const std::vector<std::vector<double>> turn_options{{0},{M_PI/2}};
  auto pole=[&](const Pose &p) {return hard(p) && !collision(p,rectangle,{{.21,.21,.005}},.002);};
  size_t qcalls=0;auto countQ=[&](const Pose &){++qcalls;return static_cast<unsigned char>(0);};
  const auto blocked=finiteHeadingSearch(turn,turn_options,radius,pole,countQ,available);
  assert(blocked.status==FiniteHeadingStatus::NoFeasible && qcalls==0);
  assert(blocked.first_rejection.reason==FiniteHeadingFailure::EdgeHard);
  assert(finiteHeadingSearch(turn,turn_options,radius,hard,countQ,available).status==FiniteHeadingStatus::Found);
  assert(qcalls==0);
  auto inner=[&](const Pose &p) {return hard(p) && !collision(p,rectangle,{{0,0,.005}},.002);};
  assert(finiteHeadingSearch(turn,turn_options,radius,inner,countQ,available).status==FiniteHeadingStatus::NoFeasible);
  auto outside=turn;for(auto &p:outside) p.x=.75;
  assert(finiteHeadingSearch(outside,turn_options,radius,hard,countQ,available).status==FiniteHeadingStatus::NoFeasible);

  // A query crossing the cooperative deadline cannot become no-feasible/PASS.
  bool in_time=true;auto after_hard=[&](const Pose &){in_time=false;return false;};
  auto time_left=[&](){return in_time;};
  auto expired=finiteHeadingSearch(turn,turn_options,radius,after_hard,countQ,time_left);
  assert(expired.status==FiniteHeadingStatus::BudgetExceeded && expired.path.empty());
  assert(expired.counts.hard_queries==1);
  in_time=true;auto after_soft=[&](const Pose &){in_time=false;return static_cast<unsigned char>(0);};
  const std::vector<Pose> translation{{0,0,0},{.004,0,0}};
  expired=finiteHeadingVerify(translation,translation,radius,hard,after_soft,time_left);
  assert(expired.status==FiniteHeadingStatus::BudgetExceeded && expired.path.empty());
  assert(expired.counts.soft_queries==1);
  assert(finiteHeadingSearch(turn,turn_options,radius,hard,countQ,[](){return false;}).status==FiniteHeadingStatus::BudgetExceeded);

  int steps=0;
  assert(!finiteHeadingSteps(INFINITY,steps));assert(!finiteHeadingSteps(NAN,steps));
  assert(!finiteHeadingSteps(.004*std::numeric_limits<int>::max(),steps));
  const double huge_radius=.004*std::numeric_limits<int>::max()/M_PI;
  const auto overflow=finiteHeadingVerify({{0,0,0},{.004,0,M_PI},{.004,0,0}},
    {{0,0,0},{.004,0,0},{.004,0,0}},huge_radius,hard,countQ,available);
  assert(overflow.status==FiniteHeadingStatus::VerificationFailed && overflow.counts.hard_queries==0 && overflow.counts.soft_queries==0);
  assert(overflow.first_rejection.reason==FiniteHeadingFailure::InvalidSteps);
  std::vector<Pose> too_many(finiteHeadingMaxPoints+1,{0,0,0});
  std::vector<std::vector<double>> too_many_options(too_many.size(),{0});
  const auto capped=finiteHeadingSearch(too_many,too_many_options,radius,hard,countQ,available);
  assert(capped.status==FiniteHeadingStatus::InvalidInput && capped.counts.hard_queries==0);

  struct Quaternion {double x,y,z,w;};
  const Quaternion q{.1,.2,.3,.9};const auto half=halfTurnWorldZ(q);
  assert(half.x==-.2 && half.y==.1 && half.z==.9 && half.w==-.3);
  assert(q.x==.1 && q.y==.2 && q.z==.3 && q.w==.9);
  assert(finiteHeadingQuaternionValid(q));assert(!finiteHeadingQuaternionValid(Quaternion{0,0,0,0}));
  assert(!finiteHeadingQuaternionValid(Quaternion{NAN,0,0,1}));
  assert(!finiteHeadingQuaternionValid(Quaternion{0,0,0,INFINITY}));
  const auto quats=finiteHeadingOrientations(q,q,q);
  assert(quats[0].x==q.x && quats[2].w==q.w && quats[3].z==half.z);
}

int main() {
  Polygon footprint{{.221,.199465},{.209465,.211},{-.210879,.211},{-.222,.199879},
                    {-.222,-.198465},{-.209465,-.211},{.210879,-.211},{.221,-.200879}};
  const MapBounds bounds{-1.,-2.,4.,2.};
  assert(footprintInsideBounds({-.75,0,0},footprint,bounds,.002));
  assert(!footprintInsideBounds({-.75,0,M_PI/4},footprint,bounds,.002));
  assert(footprintInsideBounds({3.776,0,0},footprint,bounds,.002));
  assert(!footprintInsideBounds({3.778,0,0},footprint,bounds,.002));
  assert(footprintInsideBounds({0,-1.75,M_PI/2},footprint,bounds,.002));
  assert(!footprintInsideBounds({0,-1.78,M_PI/2},footprint,bounds,.002));
  assert(!footprintInsideBounds({NAN,0,0},footprint,bounds,.002));
  assert(!footprintInsideBounds({},Polygon{},bounds,.002));
  assert(!footprintInsideBounds({},footprint,{NAN,-2.,4.,2.},.002));
  assert(!footprintInsideBounds({},footprint,{-1.,-2.,INFINITY,2.},.002));
  assert(!footprintInsideBounds({},footprint,{4.,-2.,-1.,2.},.002));
  assert(!footprintInsideBounds({},footprint,bounds,NAN));
  assert(!footprintInsideBounds({},footprint,bounds,-.002));

  std::vector<unsigned char> costs(120,0);
  PlannerMapSnapshot snapshot;
  snapshot.capture(12,10,.01,-1.,-2.,footprint,false,costs.data());
  assert(snapshot.matches(12,10,.01,-1.,-2.,footprint,false,costs.data()));
  costs[42]=1; // A cost change need not become lethal to invalidate a path.
  assert(!snapshot.matches(12,10,.01,-1.,-2.,footprint,false,costs.data()));
  snapshot.capture(12,10,.01,-1.,-2.,footprint,false,costs.data());
  costs[42]=0; // A newly cheaper alternative also requires a current search.
  assert(!snapshot.matches(12,10,.01,-1.,-2.,footprint,false,costs.data()));
  snapshot.capture(12,10,.01,-1.,-2.,footprint,false,costs.data());
  assert(!snapshot.matches(10,12,.01,-1.,-2.,footprint,false,costs.data()));
  assert(!snapshot.matches(12,10,.02,-1.,-2.,footprint,false,costs.data()));
  assert(!snapshot.matches(12,10,.01,-.99,-2.,footprint,false,costs.data()));
  assert(!snapshot.matches(12,10,.01,-1.,-2.,footprint,true,costs.data()));
  auto changed=footprint;changed[0].y+=.0001;
  assert(!snapshot.matches(12,10,.01,-1.,-2.,changed,false,costs.data()));
  snapshot.clear();
  assert(!snapshot.matches(12,10,.01,-1.,-2.,footprint,false,costs.data()));

  assert(onPoseSegment({.25,0,0},{0,0,0},{1,0,0},.25));
  assert(!onPoseSegment({.25,.001,0},{0,0,0},{1,0,0},.25));
  assert(!onPoseSegment({.25,0,.001},{0,0,0},{1,0,0},.25));
  assert(onPoseSegment({0,0,M_PI},{0,0,M_PI-.01},{0,0,-M_PI+.01},.5));

  const Polygon rectangle{{.3,.1},{-.3,.1},{-.3,-.1},{.3,-.1}};
  const MapBounds room{-1,-1,1,1};
  const double radius=std::hypot(.3,.1);
  auto hard=[&](const std::vector<Pose> &path) {return hardSweep(path,rectangle,room,{});};
  auto cost_for=[&](const std::vector<SoftCostPoint> &cells) {
    return [&,cells](const Pose &p) {
      FilledSoftCost cost(p,rectangle,.002);
      for(const auto &cell:cells) if(cost.include(cell)) break;
      return cost.cost();
    };
  };
  const std::vector<Pose> horizontal{{-.2,0,0},{.2,0,0},{.2,0,0}};
  const std::vector<Pose> vertical{{-.2,0,M_PI/2},{.2,0,M_PI/2},{.2,0,M_PI/2}};
  // A nonzero filled inflation cost is admissible when the same-XY heading
  // choice does not raise it. An all-FREE check would reject this fixture.
  auto inflation=cost_for({{{0,0,.005},176}});
  assert(inflation({0,0,0})==176);
  assert(headingOnlyPathAllowed(horizontal,horizontal,radius,hard,inflation));
  auto side_inflation=cost_for({{{0,.2,.005},176}});
  assert(headingOnlyPathAllowed(vertical,horizontal,radius,hard,side_inflation));
  assert(!headingOnlyPathAllowed(horizontal,vertical,radius,hard,side_inflation));
  auto changed_xy=horizontal;changed_xy[1].x+=1e-12;
  assert(!headingOnlyPathAllowed(horizontal,changed_xy,radius,hard,inflation));
  changed_xy=horizontal;changed_xy.pop_back();
  assert(!headingOnlyPathAllowed(horizontal,changed_xy,radius,hard,inflation));
  auto integer_difference=[](const Pose &p) {return p.yaw==0 ? 176:177;};
  assert(!headingOnlyPathAllowed(horizontal,vertical,radius,hard,integer_difference));

  // A cost increase only in an interior common sample is also rejected.
  const std::vector<Pose> short_ref{{0,0,0},{.021,0,0}};
  const std::vector<Pose> short_fixed{{0,0,.01},{.021,0,.01}};
  auto interior_cost=[](const Pose &p) {
    return p.yaw!=0 && p.x>.009 && p.x<.012 ? 177:176;
  };
  assert(!headingOnlyPathAllowed(short_ref,short_fixed,radius,hard,interior_cost));
  const std::vector<Pose> turning_ref{{0,0,0},{.004,0,M_PI/2}};
  const std::vector<Pose> turning_fixed{{0,0,0},{.004,0,0}};
  size_t common_samples=0;
  auto zero_cost=[&](const Pose &) {++common_samples;return 0;};
  assert(headingOnlyPathAllowed(turning_ref,turning_fixed,radius,hard,zero_cost));
  const auto common_steps=static_cast<size_t>(std::ceil((.004+radius*M_PI/2)/.004));
  assert(common_samples==2*(common_steps+1));

  // Pure terminal rotation contributes no translation budget, but still needs
  // a complete hard sweep: this pole misses both endpoints and hits mid-turn.
  const std::vector<Pose> stationary_ref{{0,0,0},{0,0,0}};
  const std::vector<Pose> terminal_turn{{0,0,0},{0,0,M_PI/2}};
  common_samples=0;
  assert(headingOnlyPathAllowed(stationary_ref,terminal_turn,radius,hard,zero_cost));
  assert(common_samples==0);
  const std::vector<Point> rotation_pole{{.21,.21,.005}};
  assert(!collision(terminal_turn.front(),rectangle,rotation_pole,.002));
  assert(!collision(terminal_turn.back(),rectangle,rotation_pole,.002));
  auto blocked_turn=[&](const std::vector<Pose> &p) {return hardSweep(p,rectangle,room,rotation_pole);};
  assert(!headingOnlyPathAllowed(stationary_ref,terminal_turn,radius,blocked_turn,zero_cost));
  auto inner_pole=[&](const std::vector<Pose> &p) {return hardSweep(p,rectangle,room,{{0,0,.005}});};
  assert(!headingOnlyPathAllowed(horizontal,horizontal,radius,inner_pole,zero_cost));
  auto unknown_cost=cost_for({{{0,0,.005},255}});
  assert(unknown_cost({0,0,0})==0);
  assert(!headingOnlyPathAllowed(horizontal,horizontal,radius,inner_pole,unknown_cost));
  const std::vector<Pose> outside{{.75,0,0},{.75,0,0}};
  assert(!headingOnlyPathAllowed(outside,outside,radius,hard,zero_cost));

  // Hard/unknown reference contacts never become positive soft-cost credit.
  auto no_hard_credit=cost_for({{{0,0,.005},254},{{0,0,.005},255},{{0,0,.005},1}});
  assert(no_hard_credit({0,0,0})==1);
  assert(!headingOnlyPathAllowed(horizontal,vertical,radius,hard,
    [](const Pose &p) {return p.yaw==0 ? 254:253;}));
  assert(!headingOnlyPathAllowed(horizontal,horizontal,NAN,hard,zero_cost));
  assert(!headingOnlyPathAllowed(horizontal,horizontal,-radius,hard,zero_cost));
  assert(!headingOnlyPathAllowed({}, {},radius,hard,zero_cost));
  auto nonfinite=horizontal;nonfinite.back().yaw=NAN;
  assert(!headingOnlyPathAllowed(horizontal,nonfinite,radius,hard,zero_cost));

  // The optimized soft query must use collision()'s filled / cell-radius test,
  // including interior points and near-boundary circumscribed cell contacts.
  for(const auto &p:std::vector<Pose>{{0,0,0},{0,0,M_PI/4},{.1,.05,-M_PI/2}})
    for(const auto &cell:std::vector<Point>{{0,0,.005},{.301,0,.005},{.4,0,.005},{.21,.21,.005}}) {
      FilledSoftCost query(p,rectangle,.002);query.include({cell,176});
      assert((query.cost()==176)==collision(p,rectangle,{cell},.002));
    }
  FilledSoftCost maximum({0,0,0},rectangle,.002);
  assert(!maximum.include({{0,0,.005},176}));
  assert(maximum.include({{0,0,.005},253}) && maximum.cost()==253);
  maximum.include({{0,0,.005},254});maximum.include({{0,0,.005},1});
  assert(maximum.cost()==253);
  // At small resolution, radius+resolution does not include every cell centre
  // whose circumcircle reaches the unchanged 2 mm footprint margin.
  const Polygon thin{{.3,.001},{-.3,.001},{-.3,-.001},{.3,-.001}};
  const double small_resolution=.002,thin_radius=std::hypot(.3,.001);
  const Point near_cell{.3031,0,small_resolution*std::sqrt(2.0)/2};
  assert(near_cell.x>thin_radius+small_resolution);
  assert(near_cell.x<filledQueryExtent(thin_radius,small_resolution,.002));
  assert(collision({},thin,{near_cell},.002));
  FilledSoftCost near_cost({},thin,.002);near_cost.include({near_cell,176});
  assert(near_cost.cost()==176);
  assert(filledQueryExtent(radius,.01,.002)==radius+.01);
  finiteHeadingTests();
  const auto began=SteadyHeadingBudget::Clock::now()-std::chrono::seconds(20);
  assert(!SteadyHeadingBudget(began,15.).available());
  assert(SteadyHeadingBudget(began,60.).available());
  for(double invalid:{0.,-1.,std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()})
    assert(!SteadyHeadingBudget(began,invalid).available());
  std::cout << "Actual bounds, exact cache invalidation, SE2 suffix, and filled heading-only dominance checks passed\n";
}
